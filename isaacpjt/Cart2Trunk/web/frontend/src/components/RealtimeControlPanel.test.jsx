import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { useEffect } from "react";
import { PlannerProvider, usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import RealtimeControlPanel from "./RealtimeControlPanel.jsx";
import * as client from "../api/client.js";

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

function StateProbe() {
  const state = usePlannerState();
  return (
    <div>
      <div data-testid="boxes-text">{state.boxesText}</div>
      <div data-testid="snapshot-id">{state.boxSnapshotId ?? ""}</div>
      <div data-testid="error">{state.error ? state.error.error_code : ""}</div>
    </div>
  );
}

// useResourceLoader()가 실제로는 GET /api/robot/cart-scan-files 폴링으로
// 채워주는 목록 - 이 컴포넌트 단위 테스트에서는 폴링 훅을 안 쓰므로,
// 드롭다운에 실제 <option>이 있는 상태를 재현하기 위해 리듀서 액션을
// 직접 dispatch한다.
function SeedBoxScanFiles({ files }) {
  const dispatch = usePlannerDispatch();
  useEffect(() => {
    dispatch({ type: "BOX_SCAN_FILES_REFRESHED", payload: { boxScanFiles: files } });
  }, [dispatch, files]);
  return null;
}

describe("RealtimeControlPanel", () => {
  it("renders the trunk-map and cart-scan-file dropdowns without scenario/random-generate controls", () => {
    render(<PlannerProvider><RealtimeControlPanel /></PlannerProvider>);
    expect(screen.getByText("트렁크 스캔 파일")).toBeTruthy();
    expect(screen.getByText("카트박스 스캔파일")).toBeTruthy();
    expect(screen.queryByText(/무작위/)).toBeNull();
    expect(screen.getByText("전송 (MSI2)")).toBeTruthy();
  });

  it("auto-selects and loads the newest saved file once the list appears", async () => {
    vi.spyOn(client, "fetchCartScanFileJson").mockResolvedValue({
      coordinate_frame: "m0609_base_link",
      boxes: [{ box_id: 0, corners_m: [[0, 0, 0]] }],
    });
    vi.spyOn(client, "postParseVisionCorners").mockResolvedValue({
      boxes: [{ id: "0", width: 0.5, depth: 0.35, height: 0.2, rests_on_id: null }],
      snapshot_id: "all_boxes_corners_b.json",
    });

    render(
      <PlannerProvider>
        <SeedBoxScanFiles files={["all_boxes_corners_a.json", "all_boxes_corners_b.json"]} />
        <RealtimeControlPanel />
        <StateProbe />
      </PlannerProvider>,
    );

    const select = await screen.findByTestId("box-scan-file-select");
    await waitFor(() => expect(select.value).toBe("all_boxes_corners_b.json")); // 최신(마지막) 파일 자동 선택
    await waitFor(() => expect(screen.getByTestId("snapshot-id").textContent).toBe("all_boxes_corners_b.json"));
    expect(client.fetchCartScanFileJson).toHaveBeenCalledWith("all_boxes_corners_b.json");
    expect(JSON.parse(screen.getByTestId("boxes-text").textContent)).toHaveLength(1);
  });

  it("switching the dropdown loads the newly selected file", async () => {
    vi.spyOn(client, "fetchCartScanFileJson").mockImplementation((filename) =>
      Promise.resolve({ boxes: [{ box_id: 0, corners_m: [[0, 0, 0]] }], _from: filename }));
    vi.spyOn(client, "postParseVisionCorners").mockImplementation((data) =>
      Promise.resolve({ boxes: [{ id: "0", width: 0.3, depth: 0.2, height: 0.1, rests_on_id: null }], snapshot_id: data._from }));

    render(
      <PlannerProvider>
        <SeedBoxScanFiles files={["all_boxes_corners_a.json", "all_boxes_corners_b.json"]} />
        <RealtimeControlPanel />
        <StateProbe />
      </PlannerProvider>,
    );

    const select = await screen.findByTestId("box-scan-file-select");
    await waitFor(() => expect(screen.getByTestId("snapshot-id").textContent).toBe("all_boxes_corners_b.json"));

    fireEvent.change(select, { target: { value: "all_boxes_corners_a.json" } });

    await waitFor(() => expect(screen.getByTestId("snapshot-id").textContent).toBe("all_boxes_corners_a.json"));
    expect(client.fetchCartScanFileJson).toHaveBeenCalledWith("all_boxes_corners_a.json");
  });

  it("shows a clear error when loading the selected file fails", async () => {
    vi.spyOn(client, "fetchCartScanFileJson").mockRejectedValue(new Error("network down"));

    render(
      <PlannerProvider>
        <SeedBoxScanFiles files={["all_boxes_corners_test.json"]} />
        <RealtimeControlPanel />
        <StateProbe />
      </PlannerProvider>,
    );

    await waitFor(() => expect(screen.getByTestId("error").textContent).toBe("BOX_SCAN_FILE_LOAD_FAILED"));
  });
});
