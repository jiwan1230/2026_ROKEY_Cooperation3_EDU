// src/components/PickAndPlaceTab.jsx
// "픽앤플레이스" 탭 - 로봇 카메라 실시간 + Pick&Place 진행현황(예전
// RobotControlPanel에서 트렁크/카트 Scan, 관제 로그는 "스캐닝" 탭으로 옮겼다,
// ScanningPanel.jsx 참고). 박스 단위 진행 로그는 이 탭엔 별도 로그 패널이
// 없어 화면에 쌓이진 않지만, PickPlacePanel 자체의 진행률/현재 작업
// 텍스트로 실시간 상태를 계속 보여준다.
import PickPlacePanel from "./PickPlacePanel.jsx";
import CameraPreviewPanel from "./CameraPreviewPanel.jsx";
import styles from "./PickAndPlaceTab.module.css";

export default function PickAndPlaceTab() {
  return (
    <div className={styles.panel}>
      <div className={styles.camera}><CameraPreviewPanel /></div>
      <div className={styles.pick}><PickPlacePanel /></div>
    </div>
  );
}
