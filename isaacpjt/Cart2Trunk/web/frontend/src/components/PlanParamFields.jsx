// src/components/PlanParamFields.jsx
// ControlPanel.jsx("알고리즘 검증" 탭)와 RealtimeControlPanel.jsx("실시간
// 제어" 탭)가 똑같이 쓰는 마진/우선순위/적재옵션 입력 섹션 - 원래
// ControlPanel.jsx 안에 있던 것을 그대로 옮겼다(동작/스타일 변경 없음).
// usePlannerState/usePlannerDispatch로 컨텍스트에서 직접 읽고 쓰므로 props가
// 필요 없다 - 어느 탭의 PlannerProvider 아래 렌더링되는지에 따라 자동으로
// 그 탭의 상태에 연결된다.
import { useEffect, useRef, useState } from "react";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { scenarioParams } from "./scenarioTheme.js";
import styles from "./ControlPanel.module.css";

const PREFERENCE_FIELDS = [
  { key: "entrancePreference", label: "입구 ↔ 깊은 위치", min: -1, max: 1, step: 0.1 },
  { key: "contactPreference", label: "공간활용 ↔ 안정성", min: 0, max: 2, step: 0.1 },
  { key: "heightPreference", label: "바닥부터 채우기 강도", min: 0, max: 2, step: 0.1 },
];
const DEFAULT_PREFERENCE = 1.0;

// activeScenarioId: 산업현장 시나리오(ControlPanel.jsx) 미리보기 중엔
// 우선순위 슬라이더·체크박스도 실시간 계획 값(state.params) 대신 그
// 시나리오의 고정값을 보여주고 잠근다 - "실시간 제어" 탭은 이 prop을 아예
// 안 넘기므로(undefined) 기존 동작 그대로 유지된다.
export default function PlanParamFields({ activeScenarioId } = {}) {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const scenarioLocked = Boolean(activeScenarioId);
  const locked = state.planState === "APPROVED" || scenarioLocked;
  const displayParams = scenarioLocked ? scenarioParams(activeScenarioId) : state.params;

  const setParam = (key, value) => dispatch({ type: "SET_PARAM", payload: { key, value } });

  // "개수 우선" 모드의 기본 채점 공식(트렁크 크기 기준)은 우선순위 슬라이더를
  // 아예 안 쓴다 - 그래서 슬라이더를 기본값(1.0)에서 하나라도 움직이면
  // 백엔드가 조용히 우선순위 기준 채점(weighted 공식)으로 바꿔치기한다
  // (08_unloadable_reason.py). 막지는 않되(정당한 조합 - "많이 담되 입구는
  // 신경써줘" 같은 요청), 이 전환이 일어나는 순간만 짧게 알려준다 - 계속
  // 떠있지 않고 "확인" 누르면 사라짐, 조건이 새로 발생할 때만 다시 뜬다.
  // 시나리오 미리보기 중엔 슬라이더가 항상 시나리오 고정값(기본 1.0)을
  // 보여주므로 이 배너는 필요 없다.
  const isCountFirstOverridden = !scenarioLocked && state.params.mode === "count_first" && (
    state.params.entrancePreference !== DEFAULT_PREFERENCE ||
    state.params.contactPreference !== DEFAULT_PREFERENCE ||
    state.params.heightPreference !== DEFAULT_PREFERENCE
  );
  const [noticeVisible, setNoticeVisible] = useState(false);
  const wasOverriddenRef = useRef(false);

  useEffect(() => {
    if (isCountFirstOverridden && !wasOverriddenRef.current) {
      setNoticeVisible(true);
    } else if (!isCountFirstOverridden) {
      setNoticeVisible(false);
    }
    wasOverriddenRef.current = isCountFirstOverridden;
  }, [isCountFirstOverridden]);

  return (
    <>
      {noticeVisible && (
        <div className={styles.notice} data-testid="count-first-override-notice">
          <p>
            "개수 우선" 모드에서 우선순위를 조정하면, 트렁크 크기 기준 채점
            대신 지금 설정한 우선순위 기준 채점으로 전환됩니다.
          </p>
          <button type="button" onClick={() => setNoticeVisible(false)}>확인</button>
        </div>
      )}

      <section className={styles.section}>
        <label className={styles.label}>우선순위</label>
        {PREFERENCE_FIELDS.map(({ key, label, min, max, step }) => (
          <div key={key} className={styles.fieldRow}>
            <span className={styles.fieldLabel}>{label} ({Number(displayParams[key]).toFixed(1)})</span>
            <input
              type="range"
              min={min} max={max} step={step}
              disabled={locked}
              value={displayParams[key]}
              onChange={(e) => setParam(key, Number(e.target.value))}
            />
          </div>
        ))}
      </section>

      <section className={styles.section}>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={displayParams.allowStacking}
                 onChange={(e) => setParam("allowStacking", e.target.checked)} />
          2층 이상 쌓기 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={displayParams.allowRotation}
                 onChange={(e) => setParam("allowRotation", e.target.checked)} />
          90도 회전 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={displayParams.fixedOrder}
                 onChange={(e) => setParam("fixedOrder", e.target.checked)} />
          적재 순서 고정 (박스 목록 순서 그대로)
        </label>
      </section>
    </>
  );
}
