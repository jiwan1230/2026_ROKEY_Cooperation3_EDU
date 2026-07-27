import { useState } from "react";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { usePlanActions } from "../hooks/usePlanActions.js";
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
export default function ControlPanel() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const { handleApprove, locked } = usePlanActions();
  // 무작위 생성 개수는 계산에 쓰이는 파라미터가 아니라(생성 시점에만 쓰는
  // 입력값) 전역 리듀서가 아닌 이 컴포넌트 로컬 상태로 둔다.
  const [randomBoxCount, setRandomBoxCount] = useState(6);

  const setParam = (key, value) => dispatch({ type: "SET_PARAM", payload: { key, value } });

  return (
    <div className={styles.panel}>
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
              disabled={locked}
              className={state.params.mode === value ? styles.segmentActive : styles.segment}
              onClick={() => setParam("mode", value)}
            >
              {text}
            </button>
          ))}
        </div>
      </section>

      <PlanParamFields />

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
