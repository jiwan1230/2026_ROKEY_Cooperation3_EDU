import { afterEach, describe, expect, it } from "vitest";
import { cleanup, fireEvent, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import ControlPanel from "./ControlPanel.jsx";

// vitest globals가 꺼져 있어 @testing-library/react의 자동 cleanup이 동작하지 않는다.
// (useDebouncedPlan.test.jsx와 동일한 이유로 수동 cleanup 필요)
afterEach(() => { cleanup(); });

function ModeProbe() {
  const state = usePlannerState();
  return <div data-testid="mode">{state.params.mode}</div>;
}

describe("ControlPanel", () => {
  it("selecting count_first mode updates shared state", async () => {
    render(
      <PlannerProvider>
        <ControlPanel />
        <ModeProbe />
      </PlannerProvider>,
    );
    // 모드/마진/우선순위는 "고급 설정"으로 접혀 있어서 먼저 펼쳐야 한다
    // ("처음 보는 고객"용 단순화 피드백 - 기본 접힘).
    await userEvent.click(screen.getByText(/고급 설정.*펼치기/));
    await userEvent.click(screen.getByText("개수 우선"));
    expect(screen.getByTestId("mode").textContent).toBe("count_first");
  });

  it("generating random boxes fills the box editor with 6 entries", async () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    await userEvent.click(screen.getByText("무작위 6개 생성"));
    const editor = screen.getByTestId("box-editor");
    const boxes = JSON.parse(editor.value);
    expect(boxes).toHaveLength(6);
  });

  it("adjusting the box count input changes how many random boxes are generated", async () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    const countInput = screen.getByTestId("box-count-input");
    fireEvent.change(countInput, { target: { value: "3" } });

    await userEvent.click(screen.getByText("무작위 3개 생성"));

    const editor = screen.getByTestId("box-editor");
    const boxes = JSON.parse(editor.value);
    expect(boxes).toHaveLength(3);
  });

  it("시나리오가 활성화되면 그 시나리오의 실제 파라미터(개수 우선 모드 + 마진 1cm)를 보여주고 잠근다", () => {
    render(<PlannerProvider><ControlPanel activeScenarioId="warehouse" /></PlannerProvider>);

    expect(screen.getByTestId("scenario-banner").textContent).toContain("창고/물류센터");
    expect(screen.getByDisplayValue("0.01")).toBeInTheDocument(); // 박스 간격 마진
    expect(screen.getByText("큰 것 우선")).toBeDisabled();
    expect(screen.getByText("개수 우선")).toBeDisabled();
  });

  it("시나리오가 활성화 안 됐으면 배너 없이 실시간 계획 파라미터를 그대로 조작할 수 있다", async () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    expect(screen.queryByTestId("scenario-banner")).not.toBeInTheDocument();
    await userEvent.click(screen.getByText(/고급 설정.*펼치기/));
    expect(screen.getByText("개수 우선")).not.toBeDisabled();
  });

  it("고급 설정은 기본적으로 접혀 있고, 펼치기를 누르면 모드/마진 필드가 나타난다", async () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    expect(screen.queryByText("개수 우선")).not.toBeInTheDocument();
    await userEvent.click(screen.getByText(/고급 설정.*펼치기/));
    expect(screen.getByText("개수 우선")).toBeInTheDocument();
    await userEvent.click(screen.getByText(/고급 설정.*접기/));
    expect(screen.queryByText("개수 우선")).not.toBeInTheDocument();
  });
});
