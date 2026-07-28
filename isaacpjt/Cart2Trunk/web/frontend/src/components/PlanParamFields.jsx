// src/components/PlanParamFields.jsx
// ControlPanel.jsx("알고리즘 검증" 탭)와 RealtimeControlPanel.jsx("실시간
// 제어" 탭)가 똑같이 쓰는 마진/우선순위/적재옵션 입력 섹션 - 원래
// ControlPanel.jsx 안에 있던 것을 그대로 옮겼다(동작/스타일 변경 없음).
// usePlannerState/usePlannerDispatch로 컨텍스트에서 직접 읽고 쓰므로 props가
// 필요 없다 - 어느 탭의 PlannerProvider 아래 렌더링되는지에 따라 자동으로
// 그 탭의 상태에 연결된다.
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import styles from "./ControlPanel.module.css";

const PREFERENCE_FIELDS = [
  { key: "entrancePreference", label: "입구 ↔ 깊은 위치", min: -1, max: 1, step: 0.1 },
  { key: "contactPreference", label: "공간활용 ↔ 안정성", min: 0, max: 2, step: 0.1 },
  { key: "heightPreference", label: "바닥부터 채우기 강도", min: 0, max: 2, step: 0.1 },
];

export default function PlanParamFields() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const locked = state.planState === "APPROVED";

  const setParam = (key, value) => dispatch({ type: "SET_PARAM", payload: { key, value } });

  return (
    <>
      <section className={styles.section}>
        <label className={styles.label}>우선순위</label>
        {PREFERENCE_FIELDS.map(({ key, label, min, max, step }) => (
          <div key={key} className={styles.fieldRow}>
            <span className={styles.fieldLabel}>{label} ({Number(state.params[key]).toFixed(1)})</span>
            <input
              type="range"
              min={min} max={max} step={step}
              disabled={locked}
              value={state.params[key]}
              onChange={(e) => setParam(key, Number(e.target.value))}
            />
          </div>
        ))}
      </section>

      <section className={styles.section}>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.allowStacking}
                 onChange={(e) => setParam("allowStacking", e.target.checked)} />
          2층 이상 쌓기 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.allowRotation}
                 onChange={(e) => setParam("allowRotation", e.target.checked)} />
          90도 회전 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.fixedOrder}
                 onChange={(e) => setParam("fixedOrder", e.target.checked)} />
          적재 순서 고정 (박스 목록 순서 그대로)
        </label>
      </section>
    </>
  );
}
