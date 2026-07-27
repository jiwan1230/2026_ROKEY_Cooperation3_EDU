import { afterEach, describe, expect, it, vi } from "vitest";
import { act, cleanup, fireEvent, render, screen } from "@testing-library/react";
import PickPlacePanel from "./PickPlacePanel.jsx";
import * as client from "../api/client.js";

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

describe("PickPlacePanel", () => {
  it("대기 중엔 Stop 상태와 '대기 중' 텍스트를 보여준다", () => {
    render(<PickPlacePanel />);
    expect(screen.getByTestId("pick-place-status").textContent).toBe("Stop");
    expect(screen.getByTestId("current-task").textContent).toBe("대기 중");
  });

  it("시작을 누르면 즉시 Run으로 바뀌고 '실행 중' 안내 텍스트가 보인다", async () => {
    let resolvePromise;
    vi.spyOn(client, "postPickAndPlace").mockReturnValue(
      new Promise((resolve) => { resolvePromise = resolve; })
    );

    render(<PickPlacePanel />);
    await act(async () => { fireEvent.click(screen.getByTestId("trigger-pickAndPlace")); });

    expect(screen.getByTestId("pick-place-status").textContent).toBe("Run");
    expect(screen.getByTestId("current-task").textContent).toMatch(/실행 중/);
    expect(screen.getByTestId("trigger-pickAndPlace")).toBeDisabled();
    expect(client.postPickAndPlace).toHaveBeenCalledTimes(1);

    // 정리 - act() 밖에서 pending promise가 안 남게 마무리한다.
    await act(async () => { resolvePromise({ status: "ok", boxes_placed: 4, boxes_total: 4 }); });
  });

  it("실제 백엔드 응답(성공)을 그대로 기다렸다가 완료 텍스트와 함께 Stop으로 돌아온다", async () => {
    vi.spyOn(client, "postPickAndPlace").mockResolvedValue({
      status: "ok", boxes_placed: 4, boxes_total: 4,
    });

    render(<PickPlacePanel />);
    await act(async () => { fireEvent.click(screen.getByTestId("trigger-pickAndPlace")); });

    expect(screen.getByTestId("current-task").textContent).toBe("완료 (4/4개 배치)");
    expect(screen.getByTestId("pick-place-status").textContent).toBe("Stop");
    expect(screen.getByTestId("trigger-pickAndPlace")).not.toBeDisabled();
  });

  it("백엔드가 실패를 반환하면 Warning 상태로 실패 메시지를 보여준다(가짜 완료로 안 넘어감)", async () => {
    vi.spyOn(client, "postPickAndPlace").mockRejectedValue(new Error("isaac_task_runner.py에 연결할 수 없습니다"));

    render(<PickPlacePanel />);
    await act(async () => { fireEvent.click(screen.getByTestId("trigger-pickAndPlace")); });

    expect(screen.getByTestId("current-task").textContent).toBe("isaac_task_runner.py에 연결할 수 없습니다");
    expect(screen.getByTestId("pick-place-status").textContent).toBe("Warning");
    expect(screen.getByTestId("pick-place-status").dataset.status).toBe("warning");
  });

  it("시작하면 onLog가 시작 메시지로, 실제 완료 후엔 완료 메시지로 호출된다", async () => {
    vi.spyOn(client, "postPickAndPlace").mockResolvedValue({ status: "ok", boxes_placed: 2, boxes_total: 2 });
    const onLog = vi.fn();

    render(<PickPlacePanel onLog={onLog} />);
    await act(async () => { fireEvent.click(screen.getByTestId("trigger-pickAndPlace")); });

    expect(onLog).toHaveBeenCalledWith("픽앤플레이스 시작");
    expect(onLog).toHaveBeenCalledWith("픽앤플레이스 완료");
  });
});
