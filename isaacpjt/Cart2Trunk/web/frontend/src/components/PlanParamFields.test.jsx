// src/components/PlanParamFields.test.jsx
import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import PlanParamFields from "./PlanParamFields.jsx";

afterEach(() => { cleanup(); });

function SetParam({ paramKey, value, label }) {
  const dispatch = usePlannerDispatch();
  return (
    <button onClick={() => dispatch({ type: "SET_PARAM", payload: { key: paramKey, value } })}>
      {label}
    </button>
  );
}

describe("PlanParamFields - 개수 우선 모드 우선순위 전환 알림", () => {
  it("개수 우선 모드에서 우선순위를 기본값(1.0)에서 바꾸면 알림이 뜬다", async () => {
    render(
      <PlannerProvider>
        <SetParam paramKey="mode" value="count_first" label="set-count-first" />
        <SetParam paramKey="entrancePreference" value={0.5} label="set-entrance-pref" />
        <PlanParamFields />
      </PlannerProvider>,
    );
    expect(screen.queryByTestId("count-first-override-notice")).not.toBeInTheDocument();

    await userEvent.click(screen.getByText("set-count-first"));
    await userEvent.click(screen.getByText("set-entrance-pref"));

    expect(screen.getByTestId("count-first-override-notice")).toBeInTheDocument();
  });

  it("확인 버튼을 누르면 알림이 사라진다", async () => {
    render(
      <PlannerProvider>
        <SetParam paramKey="mode" value="count_first" label="set-count-first" />
        <SetParam paramKey="entrancePreference" value={0.5} label="set-entrance-pref" />
        <PlanParamFields />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("set-count-first"));
    await userEvent.click(screen.getByText("set-entrance-pref"));
    expect(screen.getByTestId("count-first-override-notice")).toBeInTheDocument();

    await userEvent.click(screen.getByText("확인"));
    expect(screen.queryByTestId("count-first-override-notice")).not.toBeInTheDocument();
  });

  it("큰 것 우선 모드에서는 우선순위를 바꿔도 알림이 안 뜬다", async () => {
    render(
      <PlannerProvider>
        <SetParam paramKey="entrancePreference" value={0.5} label="set-entrance-pref" />
        <PlanParamFields />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("set-entrance-pref"));
    expect(screen.queryByTestId("count-first-override-notice")).not.toBeInTheDocument();
  });
});
