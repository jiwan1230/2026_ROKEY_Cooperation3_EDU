import styles from "./Header.module.css";
import { usePlannerDispatchOptional } from "../state/PlannerContext.jsx";

// onEmergencyStop: App.jsx가 탭마다 독립된 PlannerProvider를 쓰게 되면서
// (알고리즘 검증/실시간 제어가 서로 다른 상태를 갖는다), 헤더는 그 어느
// Provider에도 안 속한 채 항상 떠 있어야 한다 - 그래서 App.jsx가 "지금
// 보고 있는 탭"의 dispatch를 이 prop으로 대신 넘겨준다. prop이 없으면(예:
// Header 단위 테스트처럼 PlannerProvider 하나로 직접 감싸 렌더링하는
// 경우) 기존처럼 컨텍스트에서 바로 dispatch한다.
export default function Header({ onEmergencyStop }) {
  const contextDispatch = usePlannerDispatchOptional();
  const handleEmergencyStop = onEmergencyStop || (() => contextDispatch?.({ type: "EMERGENCY_STOP" }));

  return (
    <header className={styles.header}>
      <h1 className={styles.title}>Cart2Trunk 웹 플래너</h1>
      <button
        type="button"
        className={styles.estop}
        onClick={handleEmergencyStop}
      >
        EMERGENCY STOP
      </button>
    </header>
  );
}
