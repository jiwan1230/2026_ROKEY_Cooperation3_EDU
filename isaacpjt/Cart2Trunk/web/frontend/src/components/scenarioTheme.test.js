import { describe, expect, it } from "vitest";
import { scenarioBoxColor, scenarioParams, scenarioTrunkColor, SCENARIOS } from "./scenarioTheme.js";
import { DEFAULT_STRATEGY_PARAMS } from "../state/plannerReducer.js";

describe("SCENARIOS", () => {
  it("4개의 시나리오 id/label을 갖는다", () => {
    expect(SCENARIOS.map((s) => s.id)).toEqual(["delivery_truck", "warehouse", "cold_chain", "hazmat"]);
  });
});

describe("scenarioTrunkColor", () => {
  it("시나리오마다 서로 다른 고정 색을 반환한다", () => {
    const colors = SCENARIOS.map((s) => scenarioTrunkColor(s.id));
    expect(new Set(colors).size).toBe(4);
  });
});

describe("scenarioBoxColor", () => {
  it("창고 시나리오는 박스 id 접두사(소/대)로 색을 나눈다", () => {
    expect(scenarioBoxColor("warehouse", "소0")).toBe("#F2C94C");
    expect(scenarioBoxColor("warehouse", "대1")).toBe("#2F80ED");
  });

  it("위험물 시나리오는 hazard 종류별로 다른 경고색을 쓴다", () => {
    expect(scenarioBoxColor("hazmat", "산화제_드럼1")).toBe("#F2994A");
    expect(scenarioBoxColor("hazmat", "인화물_드럼1")).toBe("#EB5757");
    expect(scenarioBoxColor("hazmat", "일반박스1")).toBe("#9CA3AF");
  });

  it("택배/냉동 시나리오는 박스 id와 무관하게 단일 테마색을 쓴다", () => {
    expect(scenarioBoxColor("delivery_truck", "정류장1_박스")).toBe("#5A6472");
    expect(scenarioBoxColor("delivery_truck", "정류장4_박스")).toBe("#5A6472");
    expect(scenarioBoxColor("cold_chain", "냉동박스0")).toBe("#56CCF2");
  });
});

describe("scenarioParams", () => {
  it("delivery_truck은 fixedOrder + 안정성 우선순위(2.0)를 켠다", () => {
    expect(scenarioParams("delivery_truck")).toEqual({
      ...DEFAULT_STRATEGY_PARAMS, fixedOrder: true, contactPreference: 2.0,
    });
  });

  it("warehouse는 count_first 모드 + 마진 1cm만 다르고 우선순위는 기본값이다", () => {
    expect(scenarioParams("warehouse")).toEqual({ ...DEFAULT_STRATEGY_PARAMS, mode: "count_first", margin: 0.01 });
  });

  it("cold_chain은 마진 5cm + 입구 선호(-0.3)를 쓴다", () => {
    expect(scenarioParams("cold_chain")).toEqual({
      ...DEFAULT_STRATEGY_PARAMS, margin: 0.05, entrancePreference: -0.3,
    });
  });

  it("hazmat은 안정성 우선순위(1.8)만 다르다", () => {
    expect(scenarioParams("hazmat")).toEqual({ ...DEFAULT_STRATEGY_PARAMS, contactPreference: 1.8 });
  });

  it("알 수 없는 id는 기본값 그대로다", () => {
    expect(scenarioParams("nonexistent")).toEqual(DEFAULT_STRATEGY_PARAMS);
  });
});
