// src/components/scenarioTheme.js
// 산업현장 시나리오 4개의 메타데이터 + 3D 뷰어 테마 색상 - 순수 함수라
// Scene3DViewer.jsx(<Canvas> 포함, jsdom 미검증)와 분리해서 여기서 직접
// 테스트한다. id는 백엔드 routes/scenarios.py의 SCENARIO_DEFS 키와 정확히
// 일치해야 한다.
import { DEFAULT_STRATEGY_PARAMS } from "../state/plannerReducer.js";
// description은 SummaryCard의 시나리오 안내 영역에 그대로 표시된다 - 문구
// 자체를 web/backend/routes/scenarios.py의 SCENARIO_DEFS 파라미터 선택과
// 짝 맞춰서 써둔다(값이 바뀌면 여기 설명도 같이 바꿔야 함).
export const SCENARIOS = [
  {
    id: "delivery_truck", label: "택배 배송 트럭",
    description: "여러 배송지를 도는 택배 트럭 - 문을 열자마자 첫 배송지 물건이 바로 손에 닿아야 합니다. 그래서 나중 배송지 박스부터 먼저 싣는 순서(LIFO)를 고정으로 적용했고, 운행 중 흔들림에 대비해 공간활용↔안정성 우선순위를 안정성 쪽 최대(2.0)로 맞췄습니다.",
  },
  {
    id: "warehouse", label: "창고/물류센터",
    description: "입고된 박스를 최대한 많이 쟁여두는 창고 - 입구 접근성보다 공간활용이 우선이라 개수 우선(count_first) 모드 + 마진 1cm(기본 2cm보다 타이트)로 최대한 빽빽하게 채웠습니다. (우선순위 슬라이더는 일부러 기본값 그대로 뒀어요 - 건드리면 오히려 개수 우선 전용 점수 계산이 풀려서 더 적게 들어갑니다.)",
  },
  {
    id: "cold_chain", label: "냉동/냉장 물류",
    description: "냉동/냉장 컨테이너 - 박스 사이·벽 사이로 찬 공기가 순환해야 전체가 고르게 냉각됩니다. 그래서 마진을 기본(2cm)보다 훨씬 넓은 5cm로 고정했고, 유통기한 회전율 관리를 위해 입구↔깊은위치 우선순위를 입구 쪽(-0.3)으로 살짝 기울였습니다.",
  },
  {
    id: "hazmat", label: "위험물 창고",
    description: "산화제·인화물처럼 서로 반응하면 위험한 물질을 함께 보관하는 창고 - 비호환 물질끼리는 최소 안전거리 이상 떨어뜨리는 하드 규칙이 핵심이고, 드럼통이 넘어지지 않도록 공간활용↔안정성 우선순위도 안정성 쪽(1.8)으로 높였습니다.",
  },
];

const TRUNK_COLORS = {
  delivery_truck: "#8A8F98",
  warehouse: "#F2C94C",
  cold_chain: "#2D9CDB",
  hazmat: "#F2994A",
};

export function scenarioTrunkColor(scenarioId) {
  return TRUNK_COLORS[scenarioId] || "#B8B8C4";
}

export function scenarioBoxColor(scenarioId, boxId) {
  if (scenarioId === "warehouse") {
    return boxId.startsWith("대") ? "#2F80ED" : "#F2C94C";
  }
  if (scenarioId === "hazmat") {
    if (boxId.startsWith("산화제")) return "#F2994A";
    if (boxId.startsWith("인화물")) return "#EB5757";
    return "#9CA3AF";
  }
  if (scenarioId === "cold_chain") return "#56CCF2";
  if (scenarioId === "delivery_truck") return "#5A6472";
  return "#4A90D9";
}

// ControlPanel의 우선순위 슬라이더/마진/모드가 시나리오 미리보기 중에도
// 실시간 계획 값(state.params)만 계속 보여줘서 "파라미터가 진짜 적용된
// 건지 안 보인다"는 피드백을 받았다 - web/backend/routes/scenarios.py의
// SCENARIO_DEFS가 실제로 쓰는 값과 정확히 같은 값을 여기 적어두고,
// ControlPanel이 시나리오 활성화 중엔 state.params 대신 이 값을 보여주며
// 잠그게 한다(값이 바뀌면 여기도 같이 바꿔야 함 - 백엔드가 유일한 소스,
// 여긴 화면 표시용 사본).
const SCENARIO_PARAM_OVERRIDES = {
  // LIFO는 fixed_order로 구현 - "적재 순서 고정" 체크박스가 실제로 켜진
  // 상태를 반영한다. + 운행 중 흔들림 대비 안정성 우선(2.0, 최대치).
  delivery_truck: { fixedOrder: true, contactPreference: 2.0 },
  // 개수 우선 모드 + 마진 1cm(기본 2cm보다 타이트). 우선순위는 일부러
  // 기본값 그대로(위 SCENARIOS의 warehouse description 참고 - 건드리면
  // count_first 전용 밀도 점수가 풀려버림, web/backend/routes/scenarios.py
  // 주석과 동일한 이유).
  warehouse: { mode: "count_first", margin: 0.01 },
  // 마진 5cm(기본 2cm) + 회전율 관리를 위해 입구 쪽 선호(-0.3).
  cold_chain: { margin: 0.05, entrancePreference: -0.3 },
  // 안전거리 하드컷(핵심) + 드럼통 전도 방지용 안정성 우선(1.8).
  hazmat: { contactPreference: 1.8 },
};

export function scenarioParams(scenarioId) {
  return { ...DEFAULT_STRATEGY_PARAMS, ...(SCENARIO_PARAM_OVERRIDES[scenarioId] || {}) };
}
