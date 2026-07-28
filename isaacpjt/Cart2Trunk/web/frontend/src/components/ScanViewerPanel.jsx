// src/components/ScanViewerPanel.jsx
// 트렁크 Scan / 카트 Scan 칸 - 뼈대(토글+상태+버튼+3D 캔버스)가 완전히
// 같아서 kind prop으로 내용만 갈아끼운다. 시뮬레이터 탭의 PlannerContext와
// 무관하게 독립적으로 동작한다.
import { useEffect, useState } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { Matrix4 } from "three";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { postCartScan, postTrunkScan, fetchScanFile } from "../api/client.js";
import {
  TrunkWireframe, CartWireframe, SceneBoxMesh, BoundingBoxWireframe,
  layoutStagingBoxes, computeCartFootprint,
} from "./sceneMeshes.jsx";
import { DUMMY_TRUNK, DUMMY_CART_BOXES } from "./robotDummyData.js";
import styles from "./ScanViewerPanel.module.css";

const KIND_LABELS = { trunk: "트렁크 스캔", cart: "카트 스캔" };
const STATUS_TEXT = { idle: "대기", running: "진행중", done: "완료" };

// callTrigger처럼 호출 시점에 postTrunkScan/postCartScan을 직접 참조한다 -
// 미리 객체에 캡쳐해두면 vi.spyOn 목이 반영 안 되는 문제(RobotControlPanel.jsx
// 참고)를 피하기 위함.
function callScanTrigger(kind) {
  return kind === "trunk" ? postTrunkScan() : postCartScan();
}

function RawTrunkPreview() {
  return (
    <BoundingBoxWireframe x={0} y={0} z={0}
      width={DUMMY_TRUNK.width} depth={DUMMY_TRUNK.depth} height={DUMMY_TRUNK.height} />
  );
}

function ProcessedTrunkPreview() {
  return <TrunkWireframe trunk={DUMMY_TRUNK} />;
}

// 실제 ROS2 액션으로 받은 스캔 PLY(트렁크/카트 둘 다 binary_little_endian
// float32 xyz, PLYLoader가 그대로 파싱 가능)를 로드해서 점군으로 렌더링한다 -
// "원본"/"전처리" 두 토글 다 이 컴포넌트를 쓰고, url만 다르게 넘긴다
// (raw_url vs url/ply_url - trunk_scan_action_server.py/cart_scan_action_server.py가
// 둘 다 청크로 보내준다, 2026-07-28).
function RealPointCloud({ url }) {
  const [geometry, setGeometry] = useState(null);
  // 예전엔 카메라가 [1.2, 1.0, 1.6] 고정이라, 스캔 스케일/위치에 따라 매번
  // 사용자가 손으로 확대/이동해야 잘 보였다("계속 확대해서 봐야 하는" 문제)
  // - 실제 데이터가 로드되면 그 bounding sphere에 맞춰 카메라 위치와
  // OrbitControls target을 자동으로 다시 잡아서 바로 전체가 보이게 한다.
  const { camera, controls } = useThree();

  useEffect(() => {
    let cancelled = false;
    fetchScanFile(url).then((buffer) => {
      if (cancelled) return;
      const loaded = new PLYLoader().parse(buffer);
      // ROS/아이작심 좌표계(X 전방, Y 좌우, Z 위)를 Three.js 좌표계(X 화면
      // 가로, Y 위, Z 화면 깊이)로 옮긴다. 단순히 한 축만 돌리면(X축 -90도)
      // 트렁크의 실제 좌우 폭(ROS Y)이 화면 "깊이" 방향으로 가버려서 비스듬히
      // 누운 것처럼 보인다 - 좌우 폭이 화면 가로로 오도록 세 축을 순환
      // 치환한다: three_x=ros_y(좌우->가로), three_y=ros_z(위->위),
      // three_z=ros_x(전후->깊이). 순환 치환이라 손대칭이 안 바뀐다(rotation).
      loaded.applyMatrix4(new Matrix4().set(
        0, 1, 0, 0,
        0, 0, 1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1,
      ));
      // 회전 후 최저점이 그리드(y=0) 아래로 내려갈 수 있어(원점이 트렁크
      // 바닥과 정확히 일치하지 않음) - 최저점을 0에 맞춰서 바닥에 붙인다.
      loaded.computeBoundingBox();
      const minY = loaded.boundingBox.min.y;
      if (minY < 0) loaded.translate(0, -minY, 0);
      loaded.computeBoundingSphere();
      setGeometry(loaded);

      const sphere = loaded.boundingSphere;
      if (sphere && camera) {
        // fov 기준으로 bounding sphere 전체가 화면에 들어오는 거리를 구하고,
        // 여유를 좀 둔다(1.7배) - 딱 맞으면 점군 가장자리가 화면 끝에
        // 붙어서 잘려 보이기 쉽다.
        const fitDistance = (sphere.radius * 1.7) / Math.sin((camera.fov * Math.PI) / 360);
        const [ux, uy, uz] = [0.9, 0.7, 1.1];
        const ulen = Math.hypot(ux, uy, uz);
        camera.position.set(
          sphere.center.x + (ux / ulen) * fitDistance,
          sphere.center.y + (uy / ulen) * fitDistance,
          sphere.center.z + (uz / ulen) * fitDistance,
        );
        camera.near = Math.max(0.01, fitDistance / 100);
        camera.far = fitDistance * 100;
        camera.updateProjectionMatrix();
        if (controls) {
          controls.target.set(sphere.center.x, sphere.center.y, sphere.center.z);
          controls.update();
        }
      }
    });
    return () => {
      cancelled = true;
    };
  }, [url, camera, controls]);

  if (!geometry) return null;
  return (
    <points geometry={geometry}>
      <pointsMaterial size={0.01} sizeAttenuation color="#4A90D9" />
    </points>
  );
}

function RawCartPreview() {
  const layout = layoutStagingBoxes(DUMMY_CART_BOXES, DUMMY_TRUNK);
  const footprint = computeCartFootprint(layout);
  if (!footprint) return null;
  const { minX, maxX, minY, maxY, height } = footprint;
  return <BoundingBoxWireframe x={minX} y={minY} z={0} width={maxX - minX} depth={maxY - minY} height={height} />;
}

function ProcessedCartPreview() {
  const layout = layoutStagingBoxes(DUMMY_CART_BOXES, DUMMY_TRUNK);
  const footprint = computeCartFootprint(layout);
  return (
    <>
      {footprint && <CartWireframe footprint={footprint} entranceNearX={DUMMY_TRUNK.entrance_near_x} />}
      {layout.map((b) => (
        <SceneBoxMesh key={b.id} position={b.position} dimensions={b.dimensions} color="#4A90D9" />
      ))}
    </>
  );
}

const SCENE_CONTENT = {
  trunk: { raw: RawTrunkPreview, processed: ProcessedTrunkPreview },
  cart: { raw: RawCartPreview, processed: ProcessedCartPreview },
};

export default function ScanViewerPanel({ kind, onLog = () => {} }) {
  const [status, setStatus] = useState("idle");
  // 스캔 완료 시 사용자가 바로 알아볼 수 있는 형태(장애물/치수가 정리된
  // 전처리 결과)를 먼저 보여준다 - "원본"은 필요할 때 눌러서 보는 보조
  // 화면이라 기본값이 아니다.
  const [viewMode, setViewMode] = useState("processed");
  const [processedPlyUrl, setProcessedPlyUrl] = useState(null);
  const [rawPlyUrl, setRawPlyUrl] = useState(null);

  const handleTrigger = async () => {
    setStatus("running");
    try {
      // 트렁크/카트 스캔 둘 다 실제 ROS2 액션 결과에 전처리(url/ply_url)와
      // 원본(raw_url/raw_ply_url) PLY가 각각 따로 온다 - "원본" 토글은
      // raw_*, "전처리" 토글은 그 외 필드를 그대로 쓴다. 응답에 없으면(구버전
      // 백엔드 등) 기존 DUMMY_TRUNK/DUMMY_CART_BOXES 경로로 자연스럽게
      // 폴백한다. 카트 스캔의 json_url(적재 알고리즘용, algorism_bridge.py의
      // load_boxes_from_vision_json이 소비하는 스키마)은 백엔드가 이미
      // /api/robot/cart-scan-file/<filename>으로 서빙해두므로, 이 컴포넌트는
      // 뷰어 표시에 필요한 ply url들만 쓴다.
      const body = await callScanTrigger(kind);
      const processedUrl = kind === "trunk" ? body.url : body.ply_url;
      const rawUrl = kind === "trunk" ? body.raw_url : body.raw_ply_url;
      if (processedUrl) setProcessedPlyUrl(processedUrl);
      if (rawUrl) setRawPlyUrl(rawUrl);
      setStatus("done");
      const detail = kind === "cart" && body.box_count != null ? ` (박스 ${body.box_count}개)` : "";
      onLog(`${KIND_LABELS[kind]} 완료${detail}`);
    } catch {
      setStatus("idle");
      onLog(`[오류] ${KIND_LABELS[kind]} 요청 실패`);
    }
  };

  const activePlyUrl = viewMode === "raw" ? rawPlyUrl : processedPlyUrl;
  const showRealPointCloud = status === "done" && activePlyUrl;
  const Content = status === "done" && !showRealPointCloud ? SCENE_CONTENT[kind][viewMode] : null;

  return (
    <div className={styles.panel}>
      <div className={styles.header}>
        <span className={styles.title}>{KIND_LABELS[kind]}</span>
        <div className={styles.toggle}>
          <button type="button" data-testid={`${kind}-viewmode-raw`}
                  className={viewMode === "raw" ? styles.toggleActive : styles.toggleBtn}
                  onClick={() => setViewMode("raw")}>원본</button>
          <button type="button" data-testid={`${kind}-viewmode-processed`}
                  className={viewMode === "processed" ? styles.toggleActive : styles.toggleBtn}
                  onClick={() => setViewMode("processed")}>전처리</button>
        </div>
      </div>

      <div className={styles.canvasWrap}>
        <Canvas camera={{ position: [1.2, 1.0, 1.6], fov: 50 }}>
          <ambientLight intensity={0.7} />
          <directionalLight position={[3, 5, 3]} intensity={0.6} />
          <OrbitControls makeDefault />
          <Grid position={[0, -0.001, 0]} args={[4, 4]} cellSize={0.25} cellThickness={0.5}
                cellColor="#D8D8DC" sectionSize={1} sectionThickness={1} sectionColor="#B8B8C4"
                fadeDistance={5} fadeStrength={1.2} infiniteGrid />
          {showRealPointCloud && <RealPointCloud url={activePlyUrl} />}
          {Content && <Content />}
        </Canvas>
      </div>

      <button type="button" data-testid={`trigger-${kind}`} disabled={status === "running"} onClick={handleTrigger}>
        {KIND_LABELS[kind]}
      </button>
      <span className={styles.status} data-status={status} data-testid={`status-${kind}`}>
        {STATUS_TEXT[status]}
      </span>
    </div>
  );
}
