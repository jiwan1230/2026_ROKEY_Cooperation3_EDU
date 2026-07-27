// src/components/ScanViewerPanel.jsx
// 트렁크 Scan / 카트 Scan 칸 - 뼈대(토글+상태+버튼+3D 캔버스)가 완전히
// 같아서 kind prop으로 내용만 갈아끼운다. 시뮬레이터 탭의 PlannerContext와
// 무관하게 독립적으로 동작한다.
import { useEffect, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js";
import { postCartScan, postTrunkScan, fetchTrunkScanPly } from "../api/client.js";
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

// 실제 ROS2 액션으로 받은 트렁크 스캔 PLY(float32, xyz만)를 로드해서 점군으로
// 렌더링한다 - 카트 스캔/원본 토글은 아직 더미(SCENE_CONTENT)를 그대로 쓴다.
function RealTrunkPointCloud({ url }) {
  const [geometry, setGeometry] = useState(null);

  useEffect(() => {
    let cancelled = false;
    fetchTrunkScanPly(url).then((buffer) => {
      if (cancelled) return;
      const loaded = new PLYLoader().parse(buffer);
      // ROS/아이작심 좌표계(X 전방, Y 좌우, Z 위)를 Three.js 좌표계(Y가 위)에
      // 맞게 지오메트리 자체에 X축 -90도 회전을 구워넣는다 - 안 하면 트렁크의
      // 좌우 폭(Y)이 화면 세로축으로 그려져서 세워진 것처럼 보인다.
      loaded.rotateX(-Math.PI / 2);
      // 회전 후 최저점이 그리드(y=0) 아래로 내려갈 수 있어(원점이 트렁크
      // 바닥과 정확히 일치하지 않음) - 최저점을 0에 맞춰서 바닥에 붙인다.
      loaded.computeBoundingBox();
      const minY = loaded.boundingBox.min.y;
      if (minY < 0) loaded.translate(0, -minY, 0);
      loaded.computeBoundingSphere();
      setGeometry(loaded);
    });
    return () => {
      cancelled = true;
    };
  }, [url]);

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
  const [viewMode, setViewMode] = useState("raw");
  const [trunkScanUrl, setTrunkScanUrl] = useState(null);

  const handleTrigger = async () => {
    setStatus("running");
    try {
      // TODO(비전팀 연동 시): 카트 스캔은 아직 더미라 DUMMY_CART_BOXES를 그대로
      // 쓴다. 트렁크 스캔은 실제 ROS2 액션 결과(url)가 오면 그걸로 실제 점군을
      // 렌더링한다(RealTrunkPointCloud) - url이 없으면(더미 응답) 기존
      // DUMMY_TRUNK 경로로 자연스럽게 폴백한다.
      const body = await callScanTrigger(kind);
      if (kind === "trunk" && body.url) {
        setTrunkScanUrl(body.url);
      }
      setStatus("done");
      onLog(`${KIND_LABELS[kind]} 완료`);
    } catch {
      setStatus("idle");
      onLog(`[오류] ${KIND_LABELS[kind]} 요청 실패`);
    }
  };

  const showRealTrunkCloud =
    status === "done" && kind === "trunk" && viewMode === "processed" && trunkScanUrl;
  const Content = status === "done" && !showRealTrunkCloud ? SCENE_CONTENT[kind][viewMode] : null;

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
          <OrbitControls />
          <Grid position={[0, -0.001, 0]} args={[4, 4]} cellSize={0.25} cellThickness={0.5}
                cellColor="#D8D8DC" sectionSize={1} sectionThickness={1} sectionColor="#B8B8C4"
                fadeDistance={5} fadeStrength={1.2} infiniteGrid />
          {showRealTrunkCloud && <RealTrunkPointCloud url={trunkScanUrl} />}
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
