// src/components/SummaryCard.test.jsx
import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import SummaryCard from "./SummaryCard.jsx";

// vitest globals가 꺼져 있어 자동 cleanup이 동작하지 않는다 (다른 테스트 파일과 동일한 이유).
afterEach(() => { cleanup(); });

function Loader({ payload }) {
  const dispatch = usePlannerDispatch();
  return <button onClick={() => dispatch({ type: "COMPUTE_SUCCESS", payload })}>load</button>;
}

describe("SummaryCard", () => {
  it("shows placeholders before any plan is computed", () => {
    render(<PlannerProvider><SummaryCard /></PlannerProvider>);
    expect(screen.getByText(/① 파라미터를 입력하면 자동으로 계산됩니다/)).toBeInTheDocument();
  });

  it("shows a utilization grade badge once a plan is computed", async () => {
    const payload = {
      placed: [], log_lines: [],
      summary: { total: 3, placed: 2, unplaced: 1, utilization_pct: 25, avg_score: -0.5, calc_time_ms: 3 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("25.0%")).toBeInTheDocument();
    expect(screen.getByText("우수")).toBeInTheDocument();
  });

  it("shows a positive 0~100 종합 점수 (higher is better) when everything is placed (완주율 100%)", async () => {
    const payload = {
      log_lines: [],
      // score=-0.5 -> 기본 우선순위 범위[-1.6,1.0] 기준 pct=57.69, 완주율 2/2=100% -> 종합 57.69 -> 58점, 양호
      placed: [
        { box_id: "A", score: -0.5, score_breakdown: { formula: "weighted" } },
        { box_id: "B", score: -0.5, score_breakdown: { formula: "weighted" } },
      ],
      summary: { total: 2, placed: 2, unplaced: 0, utilization_pct: 10, avg_score: -0.5, calc_time_ms: 3 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("58점")).toBeInTheDocument();
    expect(screen.getByText("양호")).toBeInTheDocument();
    expect(screen.getByText(/적재율\(2\/2=100%\)/)).toBeInTheDocument();
    expect(screen.getByText(/배치 품질\(58점\)/)).toBeInTheDocument();
  });

  it("multiplies by completion rate when some boxes are unplaced, even if the placed ones scored perfectly", async () => {
    const payload = {
      log_lines: [],
      // score=-0.5 -> pct 57.69(양호), 완주율 2/3=66.7% -> 종합 38.46 -> 38점, 보통
      placed: [
        { box_id: "A", score: -0.5, score_breakdown: { formula: "weighted" } },
        { box_id: "B", score: -0.5, score_breakdown: { formula: "weighted" } },
      ],
      summary: { total: 3, placed: 2, unplaced: 1, utilization_pct: 25, avg_score: -0.5, calc_time_ms: 3 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("38점")).toBeInTheDocument();
    expect(screen.getByText("보통")).toBeInTheDocument();
    expect(screen.getByText(/적재율\(2\/3=67%\)/)).toBeInTheDocument();
  });

  it("shows 0점 when nothing was placed at all", async () => {
    const payload = {
      log_lines: [],
      placed: [],
      summary: { total: 3, placed: 0, unplaced: 3, utilization_pct: 0, avg_score: 0, calc_time_ms: 3 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("0점")).toBeInTheDocument();
    expect(screen.getByText(/박스를 하나도 싣지 못해 0점입니다/)).toBeInTheDocument();
  });

  it("shows a score grade for count_first_density when all placed boxes use it and trunk size is known", async () => {
    const payload = {
      log_lines: [],
      // score=1.5 -> trunk(0.85x1.25) 범위[0,11.5] 기준 pct=86.96 -> 87점, 우수
      placed: [{ box_id: "A", score: 1.5, score_breakdown: { formula: "count_first_density" } }],
      summary: { total: 1, placed: 1, unplaced: 0, utilization_pct: 10, avg_score: 1.5, calc_time_ms: 3 },
      trunk: { width: 0.85, depth: 1.25, height: 0.5 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("87점")).toBeInTheDocument();
    expect(screen.getByText("우수")).toBeInTheDocument();
  });

  it("averages per-box grades (instead of hiding) when weighted and count_first_density formulas are mixed", async () => {
    const payload = {
      log_lines: [],
      // A(weighted, 0.42)->pct 22.31, B(count_first_density, 1.5)->pct 86.96, 평균 54.6 -> 55점, 양호
      placed: [
        { box_id: "A", score: 0.42, score_breakdown: { formula: "weighted" } },
        { box_id: "B", score: 1.5, score_breakdown: { formula: "count_first_density" } },
      ],
      summary: { total: 2, placed: 2, unplaced: 0, utilization_pct: 10, avg_score: 0.96, calc_time_ms: 3 },
      trunk: { width: 0.85, depth: 1.25, height: 0.5 },
    };
    render(<PlannerProvider><Loader payload={payload} /><SummaryCard /></PlannerProvider>);
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText("55점")).toBeInTheDocument();
    expect(screen.getByText("양호")).toBeInTheDocument();
  });

  it("shows the scenario description when activeScenarioId is given", () => {
    render(<PlannerProvider><SummaryCard activeScenarioId="hazmat" /></PlannerProvider>);
    expect(screen.getByText(/위험물 창고 시나리오/)).toBeInTheDocument();
    expect(screen.getByText(/비호환 물질끼리는 최소 안전거리/)).toBeInTheDocument();
  });

  it("shows no scenario note when activeScenarioId is not given", () => {
    render(<PlannerProvider><SummaryCard /></PlannerProvider>);
    expect(screen.queryByText(/시나리오$/)).not.toBeInTheDocument();
  });
});
