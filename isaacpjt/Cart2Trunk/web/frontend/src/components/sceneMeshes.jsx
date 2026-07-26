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

  return boxSpecs.map((b, i) => {
    const col = i % numColumns;
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
  });
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

// 산업현장 시나리오는 쇼핑 카트를 안 쓴다 - 택배 배송 트럭은 화물칸+운전석+
// 바퀴가 있는 트럭 모형으로, 나머지(창고/냉동/위험물) 3개는 팔레트로
// 대기 구역 모양을 바꾼다(CartWireframe 대신 사용).
const TRUCK_WHEEL_RADIUS = 0.05;
const TRUCK_CAB_LENGTH = 0.15;

export function TruckWireframe({ footprint, entranceNearX }) {
  const { minX, maxX, minY, maxY, height } = footprint;
  const width = maxX - minX;
  const depth = maxY - minY;
  // 운전석(캡)은 화물칸에서 트렁크 입구 반대쪽(카트 손잡이와 같은 위치 -
  // 사람이 미는/모는 쪽) 끝에 붙인다.
  const cabX0 = entranceNearX ? maxX : minX - TRUCK_CAB_LENGTH;
  const wheelY = [minY + TRUCK_WHEEL_RADIUS, maxY - TRUCK_WHEEL_RADIUS];
  const wheelX = [minX + TRUCK_WHEEL_RADIUS, maxX - TRUCK_WHEEL_RADIUS];

  return (
    <group>
      {/* 화물칸 */}
      <mesh position={toThreeCenter(minX, minY, 0, width, depth, height)}>
        <boxGeometry args={[width, height, depth]} />
        <meshBasicMaterial color="#D8D8DC" wireframe />
      </mesh>
      {/* 운전석(캡) */}
      <mesh position={toThreeCenter(cabX0, minY, 0, TRUCK_CAB_LENGTH, depth, height * 0.7)}>
        <boxGeometry args={[TRUCK_CAB_LENGTH, height * 0.7, depth]} />
        <meshStandardMaterial color="#5A6472" />
      </mesh>
      {wheelX.flatMap((x) => wheelY.map((y) => (
        <mesh key={`${x}-${y}`} position={toThreeCenter(x, y, TRUCK_WHEEL_RADIUS, 0, 0, 0)}>
          <sphereGeometry args={[TRUCK_WHEEL_RADIUS, 12, 12]} />
          <meshStandardMaterial color="#2B2B2E" />
        </mesh>
      )))}
    </group>
  );
}

const PALLET_THICKNESS = 0.08;

export function PalletPlatform({ footprint }) {
  const { minX, maxX, minY, maxY } = footprint;
  const width = maxX - minX;
  const depth = maxY - minY;
  return (
    <mesh position={toThreeCenter(minX, minY, 0, width, depth, PALLET_THICKNESS)}>
      <boxGeometry args={[width, PALLET_THICKNESS, depth]} />
      <meshStandardMaterial color="#B08D57" roughness={0.9} />
    </mesh>
  );
}
