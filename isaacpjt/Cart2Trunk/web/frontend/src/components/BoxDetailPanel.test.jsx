import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import BoxDetailPanel from "./BoxDetailPanel.jsx";

// vitest globals가 꺼져 있어 @testing-library/react의 자동 cleanup이 동작하지 않는다.
// (ControlPanel.test.jsx, useDebouncedPlan.test.jsx와 동일한 이유로 수동 cleanup 필요)
afterEach(() => { cleanup(); });

const WEIGHTED_RESULT = {
  placed: [
    { box_id: "A", order: 1, position: [0.1, 0.2, 0.0], dimensions: [0.3, 0.2, 0.15],
      rotated: false, target_yaw: 0, score: 0.42, touches: 3,
      score_breakdown: { formula: "weighted", height_term: 0, contact_term: 0.25, wall_a_term: 0.3, wall_bc_term: 0.1 } },
  ],
  log_lines: [],
};

const COUNT_FIRST_RESULT = {
  placed: [
    { box_id: "B", order: 1, position: [0.1, 0.2, 0.0], dimensions: [0.3, 0.2, 0.15],
      rotated: false, target_yaw: 0, score: 1.5, touches: 1,
      score_breakdown: { formula: "count_first_density", height_term: 0.5, footprint_growth_term: 1.0 } },
  ],
  log_lines: [],
  trunk: { width: 0.85, depth: 1.25, height: 0.5 },
};

function Loader({ payload }) {
  const dispatch = usePlannerDispatch();
  return <button onClick={() => dispatch({ type: "COMPUTE_SUCCESS", payload })}>load</button>;
}

describe("BoxDetailPanel", () => {
  it("shows weighted-formula score breakdown for the selected box", async () => {
    render(
      <PlannerProvider>
        <Loader payload={WEIGHTED_RESULT} />
        <BoxDetailPanel />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText(/적재순서 1/)).toBeInTheDocument();
    // score=0.42, 기본 우선순위(1.0/1.0/1.0) 범위[-1.6, 1.0] 기준 22.3% -> 22점, 개선 필요
    expect(screen.getByText(/적재 점수 22점/)).toBeInTheDocument();
    expect(screen.getByText("개선 필요")).toBeInTheDocument();
    // 수식 분해는 "상세 보기"로 접혀 있다("처음 보는 고객"용 단순화 피드백).
    expect(screen.queryByText("-0.250")).not.toBeInTheDocument();
    await userEvent.click(screen.getByText(/점수 계산식 상세 보기/));
    expect(screen.getByText("-0.250")).toBeInTheDocument();
  });

  it("shows count-first-density formula score breakdown and a trunk-size-based grade", async () => {
    render(
      <PlannerProvider>
        <Loader payload={COUNT_FIRST_RESULT} />
        <BoxDetailPanel />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("load"));
    // score=1.5, trunk(0.85x1.25) 기준 범위[0, 11.5]에서 87% -> 87점, 우수
    expect(screen.getByText(/적재 점수 87점/)).toBeInTheDocument();
    expect(screen.getByText("우수")).toBeInTheDocument();
    expect(screen.getByText(/100점에 가까울수록 가장 좋은 자리/)).toBeInTheDocument();
    await userEvent.click(screen.getByText(/점수 계산식 상세 보기/));
    expect(screen.getByText("새 영역 확장 항(불리)")).toBeInTheDocument();
    expect(screen.getByText("1.000")).toBeInTheDocument();
  });
});
