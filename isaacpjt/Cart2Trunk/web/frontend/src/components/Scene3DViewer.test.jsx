import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import { PlannerProvider } from "../state/PlannerContext.jsx";
import Scene3DViewer from "./Scene3DViewer.jsx";

// <Canvas>는 실제 WebGL이 필요해서 jsdom에서 직접 렌더링하지 않는 프로젝트
// 관례(ScanViewerPanel.test.jsx와 동일) - showScenarios prop이 툴바(시나리오
// 버튼 영역)만 좌우하는지 검증하는 게 목적이라 3D 프리미티브는 전부
// 최소 스텁으로 대체한다.
vi.mock("@react-three/fiber", () => ({ Canvas: ({ children }) => <div data-testid="canvas">{children}</div> }));
vi.mock("@react-three/drei", () => ({ Grid: () => null, OrbitControls: () => null }));
vi.mock("./sceneMeshes.jsx", () => ({
  toThreeCenter: () => [0, 0, 0],
  TrunkWireframe: () => null,
  CartWireframe: () => null,
  SceneBoxMesh: () => null,
  BoundingBoxWireframe: () => null,
  layoutStagingBoxes: () => [],
  computeCartFootprint: () => null,
}));

afterEach(() => { cleanup(); });

describe("Scene3DViewer showScenarios prop", () => {
  it("shows the industry-scenario preview buttons by default (알고리즘 검증 탭)", () => {
    render(<PlannerProvider><Scene3DViewer /></PlannerProvider>);
    expect(screen.getByText("택배 배송 트럭")).toBeTruthy();
    expect(screen.getByText("창고/물류센터")).toBeTruthy();
    // Before/After 토글은 시나리오 여부와 무관하게 항상 있어야 한다.
    expect(screen.getByText("Before")).toBeTruthy();
    expect(screen.getByText("After")).toBeTruthy();
  });

  it("hides the industry-scenario preview buttons when showScenarios=false (실시간 제어 탭)", () => {
    render(<PlannerProvider><Scene3DViewer showScenarios={false} /></PlannerProvider>);
    expect(screen.queryByText("택배 배송 트럭")).toBeNull();
    expect(screen.queryByText("창고/물류센터")).toBeNull();
    expect(screen.queryByText("냉동/냉장 물류")).toBeNull();
    expect(screen.queryByText("위험물 창고")).toBeNull();
    // 나머지(Before/After・카메라 프리셋)는 그대로 남아있어야 한다.
    expect(screen.getByText("Before")).toBeTruthy();
    expect(screen.getByText("After")).toBeTruthy();
    expect(screen.getByText("front")).toBeTruthy();
  });
});
