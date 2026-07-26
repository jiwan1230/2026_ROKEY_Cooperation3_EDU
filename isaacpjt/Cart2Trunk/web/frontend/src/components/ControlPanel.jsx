import { useState } from "react";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { postApprove, postSend } from "../api/client.js";
import { SCENARIOS, scenarioParams } from "./scenarioTheme.js";
import VisionDataLoader from "./VisionDataLoader.jsx";
import styles from "./ControlPanel.module.css";

function generateRandomBoxes(count) {
  const boxes = [];
  for (let i = 0; i < count; i++) {
    boxes.push({
      id: `Box${i + 1}`,
      width: Math.round((0.15 + Math.random() * 0.30) * 100) / 100,
      depth: Math.round((0.15 + Math.random() * 0.25) * 100) / 100,
      height: Math.round((0.10 + Math.random() * 0.20) * 100) / 100,
    });
  }
  return boxes;
}

const MARGIN_FIELDS = [
  { key: "margin", label: "박스 간격" },
  { key: "wallMargin", label: "벽면 간격" },
  { key: "ceilingMargin", label: "천장 여유" },
  { key: "obstacleMargin", label: "장애물 간격" },
  { key: "entranceMargin", label: "입구 여유거리" },
];

const PREFERENCE_FIELDS = [
  { key: "entrancePreference", label: "입구 ↔ 깊은 위치", min: -1, max: 1, step: 0.1 },
  { key: "contactPreference", label: "공간활용 ↔ 안정성", min: 0, max: 2, step: 0.1 },
  { key: "heightPreference", label: "바닥부터 채우기 강도", min: 0, max: 2, step: 0.1 },
];

export default function ControlPanel({ activeScenarioId }) {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  // 무작위 생성 개수는 계산에 쓰이는 파라미터가 아니라(생성 시점에만 쓰는
  // 입력값) 전역 리듀서가 아닌 이 컴포넌트 로컬 상태로 둔다.
  const [randomBoxCount, setRandomBoxCount] = useState(6);

  // 시나리오 미리보기 중엔 실시간 계획 파라미터(state.params) 대신 그
  // 시나리오가 실제로 쓰는 고정값을 보여주고 전부 잠근다 - "우선순위
  // 슬라이더가 그대로라 파라미터가 적용된 게 안 보인다"는 피드백 반영.
  const activeScenario = SCENARIOS.find((s) => s.id === activeScenarioId);
  const params = activeScenario ? scenarioParams(activeScenarioId) : state.params;

  const setParam = (key, value) => dispatch({ type: "SET_PARAM", payload: { key, value } });

  const handleApprove = async () => {
    if (state.planState !== "COMPUTED" || !state.result) return;
    try {
      // 승인 시 감사 기록에 남는 parameters는 useDebouncedPlan.js가
      // postPlan()에 실제로 보낸 요청 바디와 형식이 같아야 한다(snake_case
      // 키 이름 + 마진 필드 ""→null + Number() 변환). state.params를 그대로
      // 넘기면 camelCase/빈 문자열 그대로라 실제 계산에 쓰인 값과 형식이
      // 어긋난다.
      const { params } = state;
      const approvedParameters = {
        mode: params.mode,
        margin: params.margin === "" ? null : Number(params.margin),
        wall_margin: params.wallMargin === "" ? null : Number(params.wallMargin),
        obstacle_margin: params.obstacleMargin === "" ? null : Number(params.obstacleMargin),
        ceiling_margin: params.ceilingMargin === "" ? null : Number(params.ceilingMargin),
        entrance_margin: params.entranceMargin === "" ? null : Number(params.entranceMargin),
        entrance_preference: params.entrancePreference,
        contact_preference: params.contactPreference,
        height_preference: params.heightPreference,
        allow_stacking: params.allowStacking,
        allow_rotation: params.allowRotation,
        fixed_order: params.fixedOrder,
      };
      const resp = await postApprove({
        box_snapshot_id: state.result.box_snapshot_id,
        trunk_map_id: state.result.trunk_map_id,
        parameters: approvedParameters,
        placed: state.result.placed,
      });
      dispatch({ type: "APPROVE_SUCCESS", payload: resp });
    } catch (err) {
      dispatch({ type: "COMPUTE_ERROR", payload: { error_code: err.error_code, cause: err.cause, action: err.action } });
    }
  };

  const handleSend = async () => {
    if (state.planState !== "APPROVED" || !state.pendingTask) return;
    try {
      const resp = await postSend({ task: state.pendingTask });
      dispatch({ type: "SEND_SUCCESS", payload: resp });
    } catch (err) {
      dispatch({ type: "COMPUTE_ERROR", payload: { error_code: err.error_code, cause: err.cause, action: err.action } });
    }
  };

  const locked = state.planState === "APPROVED";
  // 전략 파라미터(모드/마진/우선순위/체크박스)만 시나리오 미리보기 중엔
  // 추가로 잠근다 - 트렁크/박스 목록 선택은 실시간 계획 전용이라 시나리오
  // 미리보기와 무관하게 계속 조작 가능해야 한다.
  const paramsLocked = locked || Boolean(activeScenario);

  return (
    <div className={styles.panel}>
      {activeScenario && (
        <div className={styles.scenarioBanner} data-testid="scenario-banner">
          🔎 지금 <strong>{activeScenario.label}</strong> 시나리오를 보고 있어서, 아래 모드/마진/우선순위는 그 시나리오 전용 고정값이에요(수정하려면 3D 뷰어에서 "실시간 계획으로 돌아가기"를 누르세요).
        </div>
      )}
      <section className={styles.section}>
        <label className={styles.label}>트렁크 스캔 파일</label>
        <select
          className={styles.select}
          value={state.trunkMap}
          disabled={locked}
          onChange={(e) => dispatch({ type: "SET_TRUNK_MAP", payload: e.target.value })}
        >
          {state.trunkMaps.map((name) => <option key={name} value={name}>{name}</option>)}
        </select>

        <label className={styles.label}>카트 박스 프리셋</label>
        <select
          className={styles.select}
          value={state.boxPresetName}
          disabled={locked}
          onChange={(e) => dispatch({ type: "SELECT_PRESET", payload: e.target.value })}
        >
          {Object.keys(state.boxPresets).map((name) => <option key={name} value={name}>{name}</option>)}
        </select>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>적재 모드</label>
        <div className={styles.segmented}>
          {[["large_first", "큰 것 우선"], ["count_first", "개수 우선"]].map(([value, text]) => (
            <button
              key={value}
              type="button"
              disabled={paramsLocked}
              className={params.mode === value ? styles.segmentActive : styles.segment}
              onClick={() => setParam("mode", value)}
            >
              {text}
            </button>
          ))}
        </div>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>마진 (m, 비우면 기본값)</label>
        {MARGIN_FIELDS.map(({ key, label }) => (
          <div key={key} className={styles.fieldRow}>
            <span className={styles.fieldLabel}>{label}</span>
            <input
              type="text"
              inputMode="decimal"
              className={styles.input}
              disabled={paramsLocked}
              value={params[key]}
              onChange={(e) => setParam(key, e.target.value)}
            />
          </div>
        ))}
      </section>

      <section className={styles.section}>
        <label className={styles.label}>우선순위</label>
        {PREFERENCE_FIELDS.map(({ key, label, min, max, step }) => (
          <div key={key} className={styles.fieldRow}>
            <span className={styles.fieldLabel}>{label} ({Number(params[key]).toFixed(1)})</span>
            <input
              type="range"
              min={min} max={max} step={step}
              disabled={paramsLocked}
              value={params[key]}
              onChange={(e) => setParam(key, Number(e.target.value))}
            />
          </div>
        ))}
      </section>

      <section className={styles.section}>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={paramsLocked} checked={params.allowStacking}
                 onChange={(e) => setParam("allowStacking", e.target.checked)} />
          2층 이상 쌓기 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={paramsLocked} checked={params.allowRotation}
                 onChange={(e) => setParam("allowRotation", e.target.checked)} />
          90도 회전 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={paramsLocked} checked={params.fixedOrder}
                 onChange={(e) => setParam("fixedOrder", e.target.checked)} />
          적재 순서 고정 (박스 목록 순서 그대로)
        </label>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>박스 목록 (JSON)</label>
        <VisionDataLoader disabled={locked} />
        {state.boxSnapshotId && (
          <div className={styles.fieldRow}>
            <span className={styles.fieldLabel}>비전 스냅샷 ID</span>
            <span className={styles.fieldLabel}>{state.boxSnapshotId}</span>
          </div>
        )}
        <div className={styles.generateRow}>
          <input
            type="number"
            min={1}
            max={50}
            className={styles.boxCountInput}
            disabled={locked}
            value={randomBoxCount}
            data-testid="box-count-input"
            onChange={(e) => {
              const n = Math.round(Number(e.target.value));
              setRandomBoxCount(Number.isFinite(n) ? Math.min(50, Math.max(1, n)) : 1);
            }}
          />
          <button type="button" disabled={locked} onClick={() => dispatch({
            type: "GENERATE_RANDOM_BOXES", payload: generateRandomBoxes(randomBoxCount),
          })}>
            무작위 {randomBoxCount}개 생성
          </button>
        </div>
        <textarea
          className={styles.boxEditor}
          data-testid="box-editor"
          rows={10}
          disabled={locked}
          value={state.boxesText}
          onChange={(e) => dispatch({ type: "SET_BOXES_TEXT", payload: e.target.value })}
        />
      </section>

      <section className={styles.section}>
        <div className={styles.actions}>
          <button type="button" disabled={locked} onClick={() => dispatch({ type: "RESET_STRATEGY_DEFAULTS" })}>
            기본값으로 초기화
          </button>
          <button type="button" disabled={state.planState !== "COMPUTED"} onClick={handleApprove}>
            승인
          </button>
          <button type="button" disabled={!(state.planState === "COMPUTED" || state.planState === "APPROVED")}
                  onClick={() => dispatch({ type: "REJECT" })}>
            거부
          </button>
          <button type="button" disabled={state.planState !== "APPROVED"} onClick={handleSend}>
            MSI2로 전송
          </button>
        </div>
        {state.error && (
          <div className={styles.errorBox}>
            <strong>오류: {state.error.error_code}</strong>
            <p>{state.error.cause}</p>
            <p>권장 조치: {state.error.action}</p>
          </div>
        )}
      </section>
    </div>
  );
}
