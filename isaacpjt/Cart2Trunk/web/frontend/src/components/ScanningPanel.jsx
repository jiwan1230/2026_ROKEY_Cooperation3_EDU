// src/components/ScanningPanel.jsx
// "스캐닝" 탭 - 트렁크/카트 Scan 뷰어 두 개 + 관제 로그만 보여준다(예전
// RobotControlPanel의 카메라/Pick&Place는 "픽앤플레이스" 탭으로 옮겼다,
// PickAndPlaceTab.jsx 참고). 관제 로그 상태는 여기서 소유하고, 두 스캔
// 패널이 onLog로 보고하는 이벤트를 시간순으로 쌓는다.
import { useState } from "react";
import ScanViewerPanel from "./ScanViewerPanel.jsx";
import RobotLogPanel from "./RobotLogPanel.jsx";
import styles from "./ScanningPanel.module.css";

export default function ScanningPanel() {
  const [logs, setLogs] = useState([]);

  const appendLog = (message) => {
    const time = new Date().toLocaleTimeString("ko-KR", { hour12: false });
    setLogs((prev) => [{ time, message }, ...prev]);
  };

  return (
    <div className={styles.panel}>
      <div className={styles.trunk}><ScanViewerPanel kind="trunk" onLog={appendLog} /></div>
      <div className={styles.cart}><ScanViewerPanel kind="cart" onLog={appendLog} /></div>
      <div className={styles.log}><RobotLogPanel logs={logs} /></div>
    </div>
  );
}
