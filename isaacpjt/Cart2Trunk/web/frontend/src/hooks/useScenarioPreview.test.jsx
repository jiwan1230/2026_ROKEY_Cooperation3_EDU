// src/hooks/useScenarioPreview.test.jsx
import { afterEach, describe, expect, it, vi } from "vitest";
import { act, cleanup, render, screen, waitFor } from "@testing-library/react";
import { useScenarioPreview } from "./useScenarioPreview.js";
import * as client from "../api/client.js";

function Harness() {
  const scenario = useScenarioPreview();
  return (
    <div>
      <div data-testid="active">{scenario.activeScenarioId ?? ""}</div>
      <div data-testid="error">{scenario.scenarioError ?? ""}</div>
      <div data-testid="boxes-count">{scenario.scenarioResult ? scenario.scenarioResult.boxes.length : ""}</div>
      <button onClick={() => scenario.selectScenario("hazmat")}>select</button>
      <button onClick={() => scenario.randomizeScenario()}>randomize</button>
      <button onClick={() => scenario.exitScenario()}>exit</button>
    </div>
  );
}

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

describe("useScenarioPreview", () => {
  it("selectScenario가 성공하면 activeScenarioId/scenarioResult가 채워진다", async () => {
    vi.spyOn(client, "postScenarioPlan").mockResolvedValue({
      label: "위험물 창고", trunk: {}, boxes: [{ id: "a" }, { id: "b" }], placed: [], unloadable: [], summary: {},
    });

    render(<Harness />);
    await act(async () => { screen.getByText("select").click(); });

    expect(screen.getByTestId("active").textContent).toBe("hazmat");
    expect(screen.getByTestId("boxes-count").textContent).toBe("2");
    expect(client.postScenarioPlan).toHaveBeenCalledWith("hazmat", {});
  });

  it("randomizeScenario는 지금 활성화된 시나리오를 randomize:true로 다시 호출한다", async () => {
    vi.spyOn(client, "postScenarioPlan").mockResolvedValue({
      label: "위험물 창고", trunk: {}, boxes: [{ id: "a" }], placed: [], unloadable: [], summary: {},
    });

    render(<Harness />);
    await act(async () => { screen.getByText("select").click(); });
    await act(async () => { screen.getByText("randomize").click(); });

    expect(client.postScenarioPlan).toHaveBeenLastCalledWith("hazmat", { randomize: true });
  });

  it("아무 시나리오도 활성화 안 됐으면 randomizeScenario는 아무 일도 안 한다", async () => {
    vi.spyOn(client, "postScenarioPlan");
    render(<Harness />);
    await act(async () => { screen.getByText("randomize").click(); });
    expect(client.postScenarioPlan).not.toHaveBeenCalled();
  });

  it("실패하면 scenarioError가 채워지고 exitScenario로 초기화된다", async () => {
    const err = new Error("네트워크 오류");
    vi.spyOn(client, "postScenarioPlan").mockRejectedValue(err);

    render(<Harness />);
    await act(async () => { screen.getByText("select").click(); });
    await waitFor(() => expect(screen.getByTestId("error").textContent).toBe("네트워크 오류"));

    await act(async () => { screen.getByText("exit").click(); });
    expect(screen.getByTestId("error").textContent).toBe("");
    expect(screen.getByTestId("active").textContent).toBe("");
  });
});
