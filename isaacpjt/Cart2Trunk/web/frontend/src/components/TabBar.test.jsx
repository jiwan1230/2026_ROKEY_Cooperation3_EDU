import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen } from "@testing-library/react";
import TabBar from "./TabBar.jsx";

afterEach(() => { cleanup(); });

describe("TabBar", () => {
  it("현재 탭에 aria-current를 표시하고, 클릭하면 onSelect에 해당 키를 넘긴다", () => {
    const onSelect = vi.fn();
    render(<TabBar activeTab="verify" onSelect={onSelect} />);

    expect(screen.getByTestId("tab-verify").getAttribute("aria-current")).toBe("page");
    expect(screen.getByTestId("tab-scan").getAttribute("aria-current")).toBeNull();

    fireEvent.click(screen.getByTestId("tab-scan"));
    expect(onSelect).toHaveBeenCalledWith("scan");
  });

  it("activeTab이 realtime이면 realtime 탭에 aria-current가 표시된다", () => {
    render(<TabBar activeTab="realtime" onSelect={() => {}} />);
    expect(screen.getByTestId("tab-realtime").getAttribute("aria-current")).toBe("page");
    expect(screen.getByTestId("tab-verify").getAttribute("aria-current")).toBeNull();
    expect(screen.getByTestId("tab-scan").getAttribute("aria-current")).toBeNull();
  });

  it("activeTab이 pickplace이면 pickplace 탭에 aria-current가 표시된다", () => {
    render(<TabBar activeTab="pickplace" onSelect={() => {}} />);
    expect(screen.getByTestId("tab-pickplace").getAttribute("aria-current")).toBe("page");
    expect(screen.getByTestId("tab-verify").getAttribute("aria-current")).toBeNull();
  });
});
