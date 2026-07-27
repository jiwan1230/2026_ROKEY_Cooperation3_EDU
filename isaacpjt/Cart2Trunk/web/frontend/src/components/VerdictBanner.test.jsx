// src/components/VerdictBanner.test.jsx
import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import VerdictBanner from "./VerdictBanner.jsx";

afterEach(() => { cleanup(); });

function Loader({ payload }) {
  const dispatch = usePlannerDispatch();
  return <button onClick={() => dispatch({ type: "COMPUTE_SUCCESS", payload })}>load</button>;
}

describe("VerdictBanner", () => {
  it("renders nothing before a plan is computed", () => {
    const { container } = render(<PlannerProvider><VerdictBanner /></PlannerProvider>);
    expect(container.textContent).toBe("");
  });

  it("shows a 우수 verdict + CTA when everything is placed at the best possible score", async () => {
    const payload = {
      log_lines: [],
      placed: [{ box_id: "A", score: -1.6, score_breakdown: { formula: "weighted" } }],
      summary: { total: 1, placed: 1, unplaced: 0, utilization_pct: 20, avg_score: -1.6, calc_time_ms: 3 },
    };
    const onGoToRobotTab = vi.fn();
    render(
      <PlannerProvider>
        <Loader payload={payload} />
        <VerdictBanner onGoToRobotTab={onGoToRobotTab} />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("100점")).toBeInTheDocument();
    expect(screen.getByText("이 적재 방식은 아주 좋습니다")).toBeInTheDocument();
    expect(screen.getByText(/박스 1개를 전부 실었어요/)).toBeInTheDocument();
    expect(screen.getByText(/실린 자리들도 전부 최적이에요/)).toBeInTheDocument();

    const cta = screen.getByText(/로봇에게 적재 시작하기/);
    await userEvent.click(cta);
    expect(onGoToRobotTab).toHaveBeenCalledOnce();
  });

  it("multiplies by completion rate and reports the miss when some boxes are unplaced", async () => {
    const payload = {
      log_lines: [],
      // score=-0.5 -> pct 57.69(품질 "대체로 좋아요" 구간), 완주율 2/3 -> 종합 38.46 -> 38점, 보통
      placed: [
        { box_id: "A", score: -0.5, score_breakdown: { formula: "weighted" } },
        { box_id: "B", score: -0.5, score_breakdown: { formula: "weighted" } },
      ],
      summary: { total: 3, placed: 2, unplaced: 1, utilization_pct: 15, avg_score: -0.5, calc_time_ms: 3 },
    };
    render(
      <PlannerProvider>
        <Loader payload={payload} />
        <VerdictBanner />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("38점")).toBeInTheDocument();
    expect(screen.getByText("다시 검토해보면 더 좋아질 수 있어요")).toBeInTheDocument();
    expect(screen.getByText(/박스 2\/3개를 실었어요\(1개는 못 실었어요\)/)).toBeInTheDocument();
    expect(screen.getByText(/실린 자리들도 대체로 좋아요/)).toBeInTheDocument();
  });

  it("shows a 0점 failure message and hides the CTA when nothing was placed", async () => {
    const payload = {
      log_lines: [],
      placed: [],
      summary: { total: 3, placed: 0, unplaced: 3, utilization_pct: 0, avg_score: 0, calc_time_ms: 3 },
    };
    render(
      <PlannerProvider>
        <Loader payload={payload} />
        <VerdictBanner />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("0점")).toBeInTheDocument();
    expect(screen.getByText("이 적재 방식은 개선이 필요합니다")).toBeInTheDocument();
    expect(screen.getByText(/박스를 하나도 싣지 못했어요/)).toBeInTheDocument();
    expect(screen.queryByText(/로봇에게 적재 시작하기/)).not.toBeInTheDocument();
  });
});
