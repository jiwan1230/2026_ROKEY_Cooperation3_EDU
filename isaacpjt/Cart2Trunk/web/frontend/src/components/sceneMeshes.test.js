import { describe, expect, it } from "vitest";
import { computeCartFootprint, layoutStagingBoxes, toThreeCenter } from "./sceneMeshes.jsx";

describe("toThreeCenter", () => {
  it("maps our z-up corner coords to three.js y-up center coords", () => {
    expect(toThreeCenter(0, 0, 0, 0.4, 0.3, 0.2)).toEqual([0.2, 0.1, 0.15]);
  });

  it("keeps depth(y) mapped to three.js z axis", () => {
    const [, , threeZ] = toThreeCenter(0, 1.0, 0, 0.2, 0.2, 0.2);
    expect(threeZ).toBeCloseTo(1.1);
  });
});

describe("layoutStagingBoxes", () => {
  const trunk = { width: 1.0, depth: 1.0, height: 0.5, entrance_near_x: true };

  it("places boxes outside the entrance-side face (negative x) when entrance is near x=0", () => {
    const boxes = [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }];
    const [staged] = layoutStagingBoxes(boxes, trunk);
    expect(staged.position[0]).toBeLessThan(0); // 트렁크 밖(x<0)
  });

  it("places boxes beyond the far face (x > trunk.width) when entrance is near x=width", () => {
    const farTrunk = { ...trunk, entrance_near_x: false };
    const boxes = [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }];
    const [staged] = layoutStagingBoxes(boxes, farTrunk);
    expect(staged.position[0]).toBeGreaterThan(farTrunk.width);
  });

  it("returns an empty array when there is no trunk yet", () => {
    expect(layoutStagingBoxes([{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }], undefined)).toEqual([]);
  });

  it("places up to 6 boxes (3x2 grid) side by side on the floor without overlapping footprints", () => {
    const boxes = Array.from({ length: 6 }, (_, i) => ({ id: `B${i}`, width: 0.2, depth: 0.2, height: 0.1 }));
    const staged = layoutStagingBoxes(boxes, trunk);
    staged.forEach((b) => expect(b.position[2]).toBe(0)); // 칸이 남아있으니 전부 바닥(z=0)
    const cells = staged.map((b) => `${b.position[0]},${b.position[1]}`);
    expect(new Set(cells).size).toBe(6); // 서로 다른 칸에 배정됨
  });

  it("wraps the 7th box onto the 1st column, stacking flush on top with no gap", () => {
    // "카트에서 쌓인 박스들이 공중에 떠 보인다"는 피드백 - 칸(격자 6개)이 다 차서
    // 다시 배정되는 박스는 같은 칸의 이전 박스 위에 간격 없이 바로 얹혀야 한다.
    const boxes = Array.from({ length: 7 }, (_, i) => ({ id: `B${i}`, width: 0.2, depth: 0.2, height: 0.1 }));
    const staged = layoutStagingBoxes(boxes, trunk);
    const first = staged[0];
    const seventh = staged[6]; // 7 % 6 === 1번째 칸(0번 인덱스)으로 되돌아감
    expect(seventh.position[0]).toBe(first.position[0]);
    expect(seventh.position[1]).toBe(first.position[1]);
    expect(seventh.position[2]).toBeCloseTo(first.position[2] + first.dimensions[2]); // 딱 붙어서 쌓임
  });

  it("centers a smaller box within its grid cell when box sizes vary", () => {
    const boxes = [
      { id: "Big", width: 0.4, depth: 0.4, height: 0.1 },
      { id: "Small", width: 0.2, depth: 0.2, height: 0.1 },
    ];
    // 칸 크기는 가장 큰 박스(Big, 0.4) 기준 - Small은 자기 칸(두 번째 칸) 안에서 가운데 정렬돼야 한다.
    const [, small] = layoutStagingBoxes(boxes, trunk);
    expect(small.position[1]).toBeCloseTo(0.55); // spanOffset 0.45 + (0.4-0.2)/2 정렬 여백 0.1
  });

  // [사용자 피드백 - 2026-07-28] "medium/small 박스가 large 박스 위에 생성이
  // 되진 않잖아" - 실제 카트 스캔에서 rests_on_id(vision_adapter.py가
  // corners_m로 기하학적으로 계산)가 채워지는데, 그리드 배치가 이걸 무시하고
  // 있었다.
  it("stacks a box directly on top of the box it rests_on_id references, centered on its footprint", () => {
    const boxes = [
      { id: "Large", width: 0.5, depth: 0.35, height: 0.3, rests_on_id: null },
      { id: "Small", width: 0.2, depth: 0.15, height: 0.1, rests_on_id: "Large" },
    ];
    const [large, small] = layoutStagingBoxes(boxes, trunk);
    expect(large.position[2]).toBe(0); // 부모는 격자(바닥)에 배치
    expect(small.position[0]).toBeCloseTo(large.position[0] + (large.dimensions[0] - small.dimensions[0]) / 2);
    expect(small.position[1]).toBeCloseTo(large.position[1] + (large.dimensions[1] - small.dimensions[1]) / 2);
    expect(small.position[2]).toBeCloseTo(large.position[2] + large.dimensions[2]); // 딱 붙어서 얹힘
  });

  it("resolves multi-level stacking (box on box on box) in dependency order", () => {
    const boxes = [
      { id: "Small", width: 0.15, depth: 0.15, height: 0.08, rests_on_id: "Medium" },
      { id: "Large", width: 0.5, depth: 0.35, height: 0.3, rests_on_id: null },
      { id: "Medium", width: 0.3, depth: 0.25, height: 0.15, rests_on_id: "Large" },
    ];
    const staged = layoutStagingBoxes(boxes, trunk);
    const byId = Object.fromEntries(staged.map((b) => [b.id, b]));
    expect(byId.Medium.position[2]).toBeCloseTo(byId.Large.position[2] + byId.Large.dimensions[2]);
    expect(byId.Small.position[2]).toBeCloseTo(byId.Medium.position[2] + byId.Medium.dimensions[2]);
  });

  it("does not stack a box whose rests_on_id points outside the current box list (e.g. already loaded)", () => {
    // 이미 트렁크에 실려서 카트 목록에서 빠진 박스를 가리키면 안전하게
    // 바닥(격자)으로 취급해야 한다 - 부모가 없다고 화면에서 사라지면 안 됨.
    const boxes = [{ id: "Orphan", width: 0.2, depth: 0.2, height: 0.1, rests_on_id: "AlreadyLoaded" }];
    const [staged] = layoutStagingBoxes(boxes, trunk);
    expect(staged.position[2]).toBe(0);
  });

  it("treats boxes without rests_on_id exactly as before (no accidental stacking)", () => {
    const boxes = [
      { id: "A", width: 0.2, depth: 0.2, height: 0.1 },
      { id: "B", width: 0.2, depth: 0.2, height: 0.1, rests_on_id: null },
    ];
    const staged = layoutStagingBoxes(boxes, trunk);
    staged.forEach((b) => expect(b.position[2]).toBe(0));
    expect(staged[0].position[1]).not.toBe(staged[1].position[1]); // 서로 다른 칸(span 방향)에 배정됨
  });
});

describe("computeCartFootprint", () => {
  const trunk = { width: 0.85, depth: 1.25, height: 0.5, entrance_near_x: true };

  it("returns null when there are no boxes to stage", () => {
    expect(computeCartFootprint([])).toBeNull();
  });

  it("bounds the full staged layout with margin on every side", () => {
    const boxes = [
      { id: "Large", width: 0.5, depth: 0.35, height: 0.3 },
      { id: "Medium", width: 0.4, depth: 0.3, height: 0.25 },
    ];
    const layout = layoutStagingBoxes(boxes, trunk);
    const footprint = computeCartFootprint(layout);
    // Large: x=-0.8..-0.3, y=0..0.35 / Medium: x=-0.75..-0.35, y=0.425..0.725
    expect(footprint.minX).toBeCloseTo(-0.9); // -0.8 - 0.1 마진
    expect(footprint.maxX).toBeCloseTo(-0.2); // -0.3 + 0.1 마진
    expect(footprint.minY).toBeCloseTo(-0.1);
    expect(footprint.maxY).toBeCloseTo(0.825); // 0.725 + 0.1 마진
    expect(footprint.height).toBeCloseTo(0.42); // 가장 높은 박스(0.3) + 벽 여유(0.12)
  });
});
