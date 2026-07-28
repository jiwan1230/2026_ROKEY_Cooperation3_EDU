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
    // 적재 모드는 자주 쓰는 조작이라 고급 설정 밖으로 뺐다(2026-07-28) - 바로 클릭 가능.
    await userEvent.click(screen.getByText("개수 우선"));
    expect(screen.getByTestId("mode").textContent).toBe("count_first");
  });

  it("generating random boxes fills the box editor with 6 entries", async () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    // 무작위 생성 버튼은 고급 설정 밖으로 뺐지만(2026-07-28), 생성된 JSON을
    // 보여주는 텍스트 편집기(box-editor)는 여전히 "박스 목록" 섹션 안에
    // 있어 고급 설정을 펼쳐야 보인다.
    await userEvent.click(screen.getByText("무작위 6개 생성"));
    await userEvent.click(screen.getByTestId("advanced-toggle"));
    const editor = screen.getByTestId("box-editor");
    const boxes = JSON.parse(editor.value);
    expect(boxes).toHaveLength(6);
  });

  it("adjusting the box count input changes how many random boxes are generated", async () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    const countInput = screen.getByTestId("box-count-input");
    fireEvent.change(countInput, { target: { value: "3" } });

    await userEvent.click(screen.getByText("무작위 3개 생성"));
    await userEvent.click(screen.getByTestId("advanced-toggle"));

    const editor = screen.getByTestId("box-editor");
    const boxes = JSON.parse(editor.value);
    expect(boxes).toHaveLength(3);
  });

  it("locks mode/random-generate and shows a banner while a scenario is active", async () => {
    render(<PlannerProvider><ControlPanel activeScenarioId="warehouse" /></PlannerProvider>);
    expect(screen.getByTestId("scenario-banner").textContent).toContain("창고/물류센터");
    // 시나리오 고정값(count_first)이 실시간 계획 값(기본 large_first) 대신 표시된다.
    expect(screen.getByText("개수 우선").className).toMatch(/segmentActive/);
    expect(screen.getByText("개수 우선")).toBeDisabled();
    expect(screen.getByText("큰 것 우선")).toBeDisabled();
    expect(screen.getByTestId("box-count-input")).toBeDisabled();
  });

  it("does not show the scenario banner when no scenario is active", () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    expect(screen.queryByTestId("scenario-banner")).not.toBeInTheDocument();
    expect(screen.getByText("큰 것 우선")).not.toBeDisabled();
  });
});
