// src/App.jsx
import { useCallback, useRef, useState } from "react";
import { PlannerProvider, usePlannerDispatch } from "./state/PlannerContext.jsx";
import { useResourceLoader } from "./hooks/useResourceLoader.js";
import { useDebouncedPlan } from "./hooks/useDebouncedPlan.js";
import Header from "./components/Header.jsx";
import TabBar from "./components/TabBar.jsx";
import ControlPanel from "./components/ControlPanel.jsx";
import RealtimeControlPanel from "./components/RealtimeControlPanel.jsx";
import SummaryCard from "./components/SummaryCard.jsx";
import Scene3DViewer from "./components/Scene3DViewer.jsx";
import BoxDetailPanel from "./components/BoxDetailPanel.jsx";
import LogPanel from "./components/LogPanel.jsx";
import ScanningPanel from "./components/ScanningPanel.jsx";
import PickAndPlaceTab from "./components/PickAndPlaceTab.jsx";
import styles from "./App.module.css";

// Header의 EMERGENCY STOP은 PlannerProvider 밖(App() 최상단)에서 렌더링돼
// dispatch에 직접 접근할 수 없다 - 이 컴포넌트를 각 탭의 PlannerProvider
// 안에 하나씩 심어서, 그 탭의 실제 dispatch 함수를 ref로 상위에 등록해준다
// (렌더링되는 건 없음).
function DispatchRegistrar({ registerRef }) {
  const dispatch = usePlannerDispatch();
  registerRef.current = dispatch;
  return null;
}

// "알고리즘 검증"과 "실시간 제어" 탭은 컨트롤 패널(더미 생성 vs 실제 파일
// 로딩)만 다르고, 그 뒤(요약 카드/3D 뷰어/박스 상세/결과 로그)는 완전히
// 동일한 레이아웃・로직을 각자의 독립된 PlannerProvider 상태로 재사용한다.
function PlanningBody({ ControlComponent, showScenarios }) {
  // 폴링/디바운스 계산 훅은 탭 전환과 무관하게 항상 켜둔다 - 이 컴포넌트가
  // 감싸는 PlannerProvider 인스턴스 전용이라(아래 App() 참고, 탭마다 서로
  // 다른 PlannerProvider), "알고리즘 검증"에서 고른 값과 "실시간 제어"에서
  // 고른 값이 서로 안 섞이면서도, 둘 다 안 보고 있는 동안에도 계속
  // 계산/갱신된다(예전 PlannerLayout의 "탭 전환마다 리셋되지 않는다" 의도를
  // 탭 2개로 확장).
  useResourceLoader();
  useDebouncedPlan();

  return (
    <div className={styles.body}>
      <ControlComponent />
      {/* 사용자 손그림 피드백 - 요약 카드("화면 1")는 좁게, 3D 뷰어("메인
          화면 - 2")는 넓게 나란히 배치한다. 예전엔 세로로 쌓아서 요약
          카드가 위쪽 공간을 다 차지하고 3D 뷰어(툴바+캔버스)가 스크롤을
          내려야 보였다. 요약 카드는 내용이 짧아 3D 뷰어 높이만큼의 빈
          공간이 아래에 남는데, 그 자리에 결과 로그(예전엔 페이지 맨
          아래 별도 줄)를 넣어달라는 피드백을 반영해 왼쪽 칸 안에
          같이 쌓는다. */}
      <div className={styles.resultArea}>
        <div className={styles.topRow}>
          <div className={styles.leftColumn}>
            <SummaryCard />
          </div>
          <Scene3DViewer showScenarios={showScenarios} />
        </div>
        <div className={styles.bottomRow}>
          <BoxDetailPanel />
          <LogPanel />
        </div>
      </div>
    </div>
  );
}

// activeTab과 무관하게 세 탭 전부 항상 마운트해두고 CSS로만 숨긴다(조건부
// 렌더링으로 언마운트하지 않음) - 예전엔 로봇 제어 탭(트렁크/카트 스캔 3D
// 뷰어, 진행 로그)이 다른 탭에 갔다 오면 통째로 사라졌다(App.jsx가
// {activeTab === "robot" && <RobotControlPanel/>} 식으로 언마운트했기
// 때문에 컴포넌트 로컬 state가 전부 날아감 - 사용자 피드백: "다른 탭
// 갔다오면 사라지는 느낌쓰? 연결이 끊어지면 사라지나?"). 같은 방식을 세
// 탭 모두에 적용하면 "알고리즘 검증"/"실시간 제어"도 각자 독립적으로 계속
// 살아있는 세션처럼 동작한다.
function TabSlot({ active, children }) {
  return <div style={{ display: active ? "contents" : "none" }}>{children}</div>;
}

export default function App() {
  // 실제 작업 흐름(스캔 -> 계획 -> 적재) 순서를 그대로 따라 "스캐닝" 탭을
  // 기본 화면으로 연다 - "알고리즘 검증"은 개발/검증용이라 더 이상 첫
  // 화면이 아니다(TabBar.jsx 참고).
  const [activeTab, setActiveTab] = useState("scan");
  // Header의 EMERGENCY STOP이 "지금 보고 있는 탭"의 계획을 취소하도록,
  // DispatchRegistrar가 채워주는 각 탭의 dispatch를 ref로 들고 있다가
  // activeTab 기준으로 골라 쓴다. "로봇 제어" 탭은 PlannerContext를 아예
  // 안 쓰므로(RobotControlPanel.jsx 참고 - 취소할 "계획 승인"이라는 개념
  // 자체가 없음) 그때는 아무 것도 하지 않는다.
  const verifyDispatchRef = useRef(null);
  const realtimeDispatchRef = useRef(null);

  const handleEmergencyStop = useCallback(() => {
    const ref = activeTab === "verify" ? verifyDispatchRef : activeTab === "realtime" ? realtimeDispatchRef : null;
    ref?.current?.({ type: "EMERGENCY_STOP" });
  }, [activeTab]);

  return (
    <div className={styles.layout}>
      <Header onEmergencyStop={handleEmergencyStop} />
      <TabBar activeTab={activeTab} onSelect={setActiveTab} />
      <TabSlot active={activeTab === "scan"}>
        <ScanningPanel />
      </TabSlot>
      <TabSlot active={activeTab === "realtime"}>
        <PlannerProvider>
          <DispatchRegistrar registerRef={realtimeDispatchRef} />
          <PlanningBody ControlComponent={RealtimeControlPanel} showScenarios={false} />
        </PlannerProvider>
      </TabSlot>
      <TabSlot active={activeTab === "pickplace"}>
        <PickAndPlaceTab />
      </TabSlot>
      <TabSlot active={activeTab === "verify"}>
        <PlannerProvider>
          <DispatchRegistrar registerRef={verifyDispatchRef} />
          <PlanningBody ControlComponent={ControlPanel} showScenarios />
        </PlannerProvider>
      </TabSlot>
    </div>
  );
}
