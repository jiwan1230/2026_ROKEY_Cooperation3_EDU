import { afterEach, describe, expect, it } from "vitest";
import { cleanup, fireEvent, render, screen } from "@testing-library/react";
import CameraPreviewPanel from "./CameraPreviewPanel.jsx";

afterEach(() => { cleanup(); });

describe("CameraPreviewPanel", () => {
  it("MJPEG 스트림 img 태그를 렌더링한다", () => {
    render(<CameraPreviewPanel />);
    expect(screen.getByText("로봇 카메라 실시간")).toBeInTheDocument();
    const img = screen.getByTestId("camera-stream");
    expect(img.src).toContain("/stream?topic=/camera/rgb");
  });

  it("스트림 로드 실패 시 안내 문구와 다시 시도 버튼을 보여준다", () => {
    render(<CameraPreviewPanel />);
    fireEvent.error(screen.getByTestId("camera-stream"));

    expect(screen.getByText("카메라 스트림에 연결할 수 없음")).toBeInTheDocument();
    expect(screen.getByTestId("camera-retry")).toBeInTheDocument();
  });

  it("다시 시도를 누르면 스트림 img로 되돌아간다", () => {
    render(<CameraPreviewPanel />);
    fireEvent.error(screen.getByTestId("camera-stream"));
    fireEvent.click(screen.getByTestId("camera-retry"));

    expect(screen.getByTestId("camera-stream")).toBeInTheDocument();
  });
});
