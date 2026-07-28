// src/components/sceneMeshes.jsx
// Scene3DViewer.jsx에서 추출한 순수 3D 렌더링 부품 - props만 받고
// PlannerContext 등 전역 상태에 의존하지 않는다. 시뮬레이터 탭(Scene3DViewer)과
// 로봇 제어 탭(ScanViewerPanel)이 같이 쓴다.

const STAGING_GAP = 0.05; // 대기 박스끼리, 그리고 트렁크 입구 면과의 간격(m)
const STAGING_OFFSET = 0.3; // 트렁크 입구 면에서 대기 구역까지의 거리(m)
const CART_GRID_SPAN_COLUMNS = 3; // 카트 폭 방향(옆으로) 칸 수
const CART_GRID_DEPTH_ROWS = 2; // 카트 안쪽(입구 방향) 칸 수

export function layoutStagingBoxes(boxSpecs, trunk) {
  if (!trunk || boxSpecs.length === 0) return [];
  const entranceNearX = trunk.entrance_near_x !== false;

  const cellWidth = Math.max(...boxSpecs.map((b) => b.width));
  const cellDepth = Math.max(...boxSpecs.map((b) => b.depth));
  const numColumns = CART_GRID_SPAN_COLUMNS * CART_GRID_DEPTH_ROWS;
  const columnHeights = new Array(numColumns).fill(0);
  let nextCellIndex = 0;

  const placeInGrid = (b) => {
    const col = nextCellIndex % numColumns;
    nextCellIndex += 1;
    const depthIndex = Math.floor(col / CART_GRID_SPAN_COLUMNS);
    const spanIndex = col % CART_GRID_SPAN_COLUMNS;

    const rowDepthOffset = depthIndex * (cellWidth + STAGING_GAP);
    const spanOffset = spanIndex * (cellDepth + STAGING_GAP);

    const cellOriginX = entranceNearX
      ? -(STAGING_OFFSET + rowDepthOffset + cellWidth)
      : trunk.width + STAGING_OFFSET + rowDepthOffset;
    const x = cellOriginX + (cellWidth - b.width) / 2;
    const y = spanOffset + (cellDepth - b.depth) / 2;
    const z = columnHeights[col];
    columnHeights[col] += b.height;

    return { id: b.id, position: [x, y, z], dimensions: [b.width, b.depth, b.height] };
  };

  // [사용자 피드백] "medium/small 박스가 large 박스 위에 생성이 되진 않잖아" -
  // 예전엔 rests_on_id(어떤 박스가 뭐 위에 얹혀있는지, vision_adapter.py가 실제
  // 스캔 데이터에서 corners_m으로 기하학적으로 계산해 채워준다)를 전혀 안 보고
  // 그냥 격자 칸에 순서대로 채워 넣기만 했다 - 더미 데이터 검증용으로는 상관
  //없었지만, 실제 카트 스캔 데이터에서는 물리적으로 얹혀있는 관계를 그대로
  // 반영해야 한다. rests_on_id가 boxSpecs 안의 다른 박스를 실제로 가리키는
  // 것만 "얹힌 박스"로 취급하고(가리키는 대상이 이 목록에 없으면 - 예를 들어
  // 이미 트렁크에 실려서 카트 목록에서 빠졌으면 - 안전하게 바닥/격자 배정으로
  // 되돌린다), 부모는 항상 먼저(격자 또는 다른 부모 위에) 자리를 잡은 뒤에
  // 그 위에 자식을 얹는다.
  const byId = Object.fromEntries(boxSpecs.map((b) => [b.id, b]));
  const restingIds = new Set(
    boxSpecs.filter((b) => b.rests_on_id && byId[b.rests_on_id]).map((b) => b.id),
  );

  const layoutById = {};
  for (const b of boxSpecs) {
    if (!restingIds.has(b.id)) layoutById[b.id] = placeInGrid(b);
  }

  let remaining = boxSpecs.filter((b) => restingIds.has(b.id));
  while (remaining.length > 0) {
    const stillWaiting = [];
    let progressed = false;
    for (const b of remaining) {
      const parent = layoutById[b.rests_on_id];
      if (!parent) {
        stillWaiting.push(b);
        continue;
      }
      const [px, py, pz] = parent.position;
      const [pw, pd, ph] = parent.dimensions;
      layoutById[b.id] = {
        id: b.id,
        position: [px + (pw - b.width) / 2, py + (pd - b.depth) / 2, pz + ph],
        dimensions: [b.width, b.depth, b.height],
      };
      progressed = true;
    }
    if (!progressed) {
      // 순환 참조 등 이상 데이터 - 부모를 영영 못 찾는 박스는 격자에 바닥
      // 취급으로 배치해서 최소한 화면에서 누락되지 않게 한다.
      for (const b of stillWaiting) layoutById[b.id] = placeInGrid(b);
      break;
    }
    remaining = stillWaiting;
  }

  return boxSpecs.map((b) => layoutById[b.id]);
}

export function toThreeCenter(x, y, z, w, d, h) {
  return [x + w / 2, z + h / 2, y + d / 2];
}

const TRUNK_LID_COLOR = "#9C1F1F";
const TRUNK_TAILLIGHT_COLOR = "#C62828";
const TRUNK_LID_TILT = (32 * Math.PI) / 180;

export function TrunkWireframe({ trunk }) {
  const entranceNearX = trunk.entrance_near_x !== false;
  const outwardSign = entranceNearX ? -1 : 1;
  const entranceX = entranceNearX ? 0 : trunk.width;
  const tailX = entranceNearX ? -0.015 : trunk.width - 0.015;
  const lidLength = trunk.height * 0.8;
  const lidRotationZ = Math.PI / 2 + outwardSign * TRUNK_LID_TILT;

  return (
    <group>
      <mesh position={toThreeCenter(0, 0, 0, trunk.width, trunk.depth, trunk.height)}>
        <boxGeometry args={[trunk.width, trunk.height, trunk.depth]} />
        <meshBasicMaterial color="#6E6E73" wireframe />
      </mesh>

      {[0.08, Math.max(0.08, trunk.depth - 0.22)].map((yStart, i) => (
        <mesh key={i} position={toThreeCenter(tailX, yStart, trunk.height - 0.14, 0.03, 0.14, 0.1)}>
          <boxGeometry args={[0.03, 0.1, 0.14]} />
          <meshStandardMaterial color={TRUNK_TAILLIGHT_COLOR} emissive={TRUNK_TAILLIGHT_COLOR} emissiveIntensity={0.4} />
        </mesh>
      ))}

      <group position={toThreeCenter(entranceX, 0, trunk.height, 0, trunk.depth, 0)} rotation={[0, 0, lidRotationZ]}>
        <mesh position={[lidLength / 2, 0, 0]}>
          <boxGeometry args={[lidLength, 0.05, trunk.depth]} />
          <meshStandardMaterial color={TRUNK_LID_COLOR} roughness={0.5} metalness={0.2} />
        </mesh>
      </group>
    </group>
  );
}

const CART_MARGIN = 0.1;
const CART_WALL_CLEARANCE = 0.12;
const CART_WHEEL_RADIUS = 0.045;

export function computeCartFootprint(fullBoxLayout) {
  if (!fullBoxLayout || fullBoxLayout.length === 0) return null;
  const minX = Math.min(...fullBoxLayout.map((b) => b.position[0]));
  const maxX = Math.max(...fullBoxLayout.map((b) => b.position[0] + b.dimensions[0]));
  const maxY = Math.max(...fullBoxLayout.map((b) => b.position[1] + b.dimensions[1]));
  const maxHeight = Math.max(...fullBoxLayout.map((b) => b.dimensions[2]));
  return {
    minX: minX - CART_MARGIN,
    maxX: maxX + CART_MARGIN,
    minY: -CART_MARGIN,
    maxY: maxY + CART_MARGIN,
    height: maxHeight + CART_WALL_CLEARANCE,
  };
}

export function CartWireframe({ footprint, entranceNearX }) {
  const { minX, maxX, minY, maxY, height } = footprint;
  const width = maxX - minX;
  const depth = maxY - minY;
  const handleX = entranceNearX ? minX : maxX;
  const wheelY = [minY + CART_WHEEL_RADIUS, maxY - CART_WHEEL_RADIUS];
  const wheelX = [minX + CART_WHEEL_RADIUS, maxX - CART_WHEEL_RADIUS];

  return (
    <group>
      <mesh position={toThreeCenter(minX, minY, 0, width, depth, height)}>
        <boxGeometry args={[width, height, depth]} />
        <meshBasicMaterial color="#A2724F" wireframe />
      </mesh>
      {wheelX.flatMap((x) => wheelY.map((y) => (
        <mesh key={`${x}-${y}`} position={toThreeCenter(x, y, CART_WHEEL_RADIUS, 0, 0, 0)}>
          <sphereGeometry args={[CART_WHEEL_RADIUS, 12, 12]} />
          <meshStandardMaterial color="#3A3A3C" />
        </mesh>
      )))}
      <mesh position={toThreeCenter(handleX, minY, height * 0.75, 0, depth, 0)}>
        <boxGeometry args={[0.03, 0.03, depth]} />
        <meshStandardMaterial color="#3A3A3C" />
      </mesh>
    </group>
  );
}

export function SceneBoxMesh({ position, dimensions, color, dashed }) {
  const [w, d, h] = dimensions;
  const [x, y, z] = position;
  return (
    <group position={toThreeCenter(x, y, z, w, d, h)}>
      <mesh>
        <boxGeometry args={[w, h, d]} />
        <meshStandardMaterial color={color} transparent opacity={dashed ? 0.55 : 0.9} />
      </mesh>
      <mesh>
        <boxGeometry args={[w, h, d]} />
        <meshBasicMaterial color="#1D1D1F" wireframe />
      </mesh>
    </group>
  );
}

// "원본"(가공 전) 더미 뷰용 - 개별 물체 구분 없이 전체를 감싸는 단순
// 바운딩박스 와이어프레임 하나만 그린다.
export function BoundingBoxWireframe({ x, y, z, width, depth, height, color = "#B8B8C4" }) {
  return (
    <mesh position={toThreeCenter(x, y, z, width, depth, height)}>
      <boxGeometry args={[width, height, depth]} />
      <meshBasicMaterial color={color} wireframe />
    </mesh>
  );
}
