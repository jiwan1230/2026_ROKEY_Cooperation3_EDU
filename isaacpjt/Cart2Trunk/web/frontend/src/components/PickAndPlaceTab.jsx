// src/components/PickAndPlaceTab.jsx
// "픽앤플레이스" 탭 - 창을 위/아래로 나눠 Pick&Place 진행현황을 위에,
// 로봇 카메라 실시간을 아래에 배치한다. 카메라 화면이 더 잘 보이도록
// 아래쪽(카메라)에 더 큰 비중(2:3)을 준다(2026-07-28). 예전 RobotControlPanel에서
// 트렁크/카트 Scan, 관제 로그는 "스캐닝" 탭으로 옮겼다(ScanningPanel.jsx
// 참고). 박스 단위 진행 로그는 이 탭엔 별도 로그 패널이 없어 화면에 쌓이진
// 않지만, PickPlacePanel 자체의 진행률/현재 작업 텍스트로 실시간 상태를
// 계속 보여준다.
import PickPlacePanel from "./PickPlacePanel.jsx";
import CameraPreviewPanel from "./CameraPreviewPanel.jsx";
import styles from "./PickAndPlaceTab.module.css";

export default function PickAndPlaceTab() {
  return (
    <div className={styles.panel}>
      <div className={styles.pick}><PickPlacePanel /></div>
      <div className={styles.camera}><CameraPreviewPanel /></div>
    </div>
  );
}
