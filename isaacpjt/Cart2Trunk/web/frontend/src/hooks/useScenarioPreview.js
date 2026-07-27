// src/hooks/useScenarioPreview.js
// 산업현장 시나리오 미리보기 상태 - SummaryCard(시나리오 안내 문구)와
// Scene3DViewer(3D 미리보기)가 둘 다 필요해서 두 컴포넌트의 공통 부모
// (App.jsx의 SimulatorBody)에서 이 훅으로 한 번만 만들어 내려준다.
// PlannerContext(state.result)는 전혀 건드리지 않는 완전히 별도 상태다 -
// 지금 작업 중인 실시간 계획이 시나리오 미리보기 때문에 사라지면 안 된다.
import { useState } from "react";
import { postScenarioPlan } from "../api/client.js";

export function useScenarioPreview() {
  const [activeScenarioId, setActiveScenarioId] = useState(null);
  const [scenarioResult, setScenarioResult] = useState(null);
  const [scenarioError, setScenarioError] = useState(null);
  const [scenarioLoading, setScenarioLoading] = useState(false);

  const selectScenario = async (id, options = {}) => {
    setScenarioLoading(true);
    setScenarioError(null);
    try {
      const result = await postScenarioPlan(id, options);
      setActiveScenarioId(id);
      setScenarioResult(result);
    } catch (err) {
      setScenarioError(err.cause || err.message || "시나리오를 불러오지 못했습니다.");
    } finally {
      setScenarioLoading(false);
    }
  };

  // "무작위로 다시 생성" - 지금 보고 있는 시나리오를 그대로 유지한 채
  // 박스만 그 시나리오 성격에 맞는 무작위 세트로 바꿔서 다시 계산한다.
  const randomizeScenario = () => {
    if (!activeScenarioId) return;
    selectScenario(activeScenarioId, { randomize: true });
  };

  const exitScenario = () => {
    setActiveScenarioId(null);
    setScenarioResult(null);
    setScenarioError(null);
  };

  return {
    activeScenarioId, scenarioResult, scenarioError, scenarioLoading,
    selectScenario, randomizeScenario, exitScenario,
  };
}
