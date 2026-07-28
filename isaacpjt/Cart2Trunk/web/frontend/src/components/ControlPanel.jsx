import { useState } from "react";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { usePlanActions } from "../hooks/usePlanActions.js";
import { SCENARIOS, scenarioParams } from "./scenarioTheme.js";
import VisionDataLoader from "./VisionDataLoader.jsx";
import PlanParamFields from "./PlanParamFields.jsx";
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

// "알고리즘 검증" 탭 - 더미 프리셋/무작위 생성으로 적재 알고리즘을 검증하는
// 용도라 실제 로봇(MSI2) 전송 버튼은 없다(승인/거부까지만) - 실제 전송은
// "실시간 제어" 탭(RealtimeControlPanel.jsx)의 몫이다.
export default function ControlPanel({ activeScenarioId }) {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const { handleApprove, locked } = usePlanActions();
  // 무작위 생성 개수는 계산에 쓰이는 파라미터가 아니라(생성 시점에만 쓰는
  // 입력값) 전역 리듀서가 아닌 이 컴포넌트 로컬 상태로 둔다.
  const [randomBoxCount, setRandomBoxCount] = useState(6);

  // 시나리오 미리보기 중엔 실시간 계획 파라미터(state.params) 대신 그
  // 시나리오가 실제로 쓰는 고정값을 보여주고 전부 잠근다 - "우선순위
  // 슬라이더가 그대로라 파라미터가 적용된 게 안 보인다"는 피드백 반영.
  const activeScenario = SCENARIOS.find((s) => s.id === activeScenarioId);
  const displayParams = activeScenario ? scenarioParams(activeScenarioId) : state.params;
  // 전략 파라미터(모드/우선순위/체크박스)만 시나리오 미리보기 중엔 추가로
  // 잠근다 - 트렁크/박스 목록 선택은 실시간 계획 전용이라 시나리오
  // 미리보기와 무관하게 계속 조작 가능해야 한다.
  const paramsLocked = locked || Boolean(activeScenario);
  // 적재 모드/우선순위/체크박스/박스 목록을 항상 펼쳐두면 왼쪽 바가
  // 버튼/입력으로 꽉 차 복잡해 보인다는 피드백 - 자주 안 바꾸는 세부
  // 옵션이라 기본은 접어두고, 필요할 때만 펼친다. 마진 입력(박스/벽면/천장/
  // 장애물/입구 간격)은 아예 UI에서 제거했다(사용자 요청, 2026-07-28) -
  // 항상 기본값을 쓴다.
  const [advancedOpen, setAdvancedOpen] = useState(false);

  const setParam = (key, value) => dispatch({ type: "SET_PARAM", payload: { key, value } });

  return (
    <div className={styles.panel}>
      {activeScenario && (
        <div className={styles.scenarioBanner} data-testid="scenario-banner">
          🔎 지금 <strong>{activeScenario.label}</strong> 시나리오를 보고 있어서, 아래 모드/우선순위는 그 시나리오 전용 고정값이에요(수정하려면 3D 뷰어에서 "실시간 계획으로 돌아가기"를 누르세요).
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

      {/* 적재 모드 토글과 무작위 생성 버튼은 자주 만지는 조작이라(사용자
          요청, 2026-07-28) 고급 설정 밖으로 뺐다 - 매번 펼치지 않고도 바로
          쓸 수 있어야 한다는 피드백. */}
      <section className={styles.section}>
        <label className={styles.label}>적재 모드</label>
        <div className={styles.segmented}>
          {[["large_first", "큰 것 우선"], ["count_first", "개수 우선"]].map(([value, text]) => (
            <button
              key={value}
              type="button"
              disabled={paramsLocked}
              className={displayParams.mode === value ? styles.segmentActive : styles.segment}
              onClick={() => setParam("mode", value)}
            >
              {text}
            </button>
          ))}
        </div>
      </section>

      <div className={styles.generateRow}>
        <input
          type="number"
          min={1}
          max={50}
          className={styles.boxCountInput}
          disabled={paramsLocked}
          value={randomBoxCount}
          data-testid="box-count-input"
          onChange={(e) => {
            const n = Math.round(Number(e.target.value));
            setRandomBoxCount(Number.isFinite(n) ? Math.min(50, Math.max(1, n)) : 1);
          }}
        />
        <button type="button" disabled={paramsLocked} onClick={() => dispatch({
          type: "GENERATE_RANDOM_BOXES", payload: generateRandomBoxes(randomBoxCount),
        })}>
          무작위 {randomBoxCount}개 생성
        </button>
      </div>

      <button
        type="button"
        className={styles.advancedToggle}
        data-testid="advanced-toggle"
        onClick={() => setAdvancedOpen((open) => !open)}
      >
        고급 설정(우선순위·박스 목록 JSON) {advancedOpen ? "숨기기 ▴" : "보기 ▾"}
      </button>

      {advancedOpen && (
        <>
          <PlanParamFields activeScenarioId={activeScenarioId} />

          <section className={styles.section}>
            <label className={styles.label}>박스 목록 (JSON)</label>
            <VisionDataLoader disabled={locked} />
            {state.boxSnapshotId && (
              <div className={styles.fieldRow}>
                <span className={styles.fieldLabel}>비전 스냅샷 ID</span>
                <span className={styles.fieldLabel}>{state.boxSnapshotId}</span>
              </div>
            )}
            <textarea
              className={styles.boxEditor}
              data-testid="box-editor"
              rows={10}
              disabled={locked}
              value={state.boxesText}
              onChange={(e) => dispatch({ type: "SET_BOXES_TEXT", payload: e.target.value })}
            />
          </section>
        </>
      )}

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
