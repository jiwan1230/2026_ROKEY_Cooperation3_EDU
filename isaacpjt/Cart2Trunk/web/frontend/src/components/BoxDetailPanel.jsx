import { useState } from "react";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { gradeBoxScore } from "../utils/scoreGrading.js";
import styles from "./BoxDetailPanel.module.css";

export default function BoxDetailPanel() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  // "처음 보는 고객"에게는 점수+등급 한 줄이면 충분하고, 높이항/접촉면항
  // 같은 수식 분해는 알고리즘을 튜닝하는 엔지니어에게만 필요하다 - 기본
  // 접어두고 원할 때만 펼친다.
  const [detailOpen, setDetailOpen] = useState(false);
  const placed = state.result?.placed || [];
  const selected = placed.find((p) => p.box_id === state.selectedBoxId);
  const grade = selected
    ? gradeBoxScore(selected.score_breakdown.formula, selected.score, {
        preferences: state.params,
        trunk: state.result?.trunk,
      })
    : null;

  return (
    <div className={styles.panel}>
      <label className={styles.label}>박스 상세 조회</label>
      <select
        className={styles.select}
        value={state.selectedBoxId}
        onChange={(e) => dispatch({ type: "SELECT_BOX", payload: e.target.value })}
      >
        {placed.map((p) => (
          <option key={p.box_id} value={p.box_id}>{p.order}. {p.box_id}</option>
        ))}
      </select>

      {selected ? (
        <div className={styles.detail}>
          <p>
            <strong>{selected.box_id}</strong> · 적재순서 {selected.order} ·
            Target=({selected.position.map((v) => v.toFixed(2)).join(", ")})m ·
            Yaw={selected.target_yaw.toFixed(2)}rad
          </p>
          <p>접촉면 {selected.touches}/6개, {selected.rotated ? "90도 회전됨" : "정자세"}</p>
          {/* "몇 점이면 좋은 건지 기준이 없다" + "점수가 왜 음수냐"는 피드백 -
              algorism 원점수(낮을수록 좋음)를 그대로 보여주는 대신, gradeBoxScore가
              계산한 0~100% 위치(높을수록 좋음)를 "적재 점수"로 보여준다.
              formula별 등급 범위는 gradeBoxScore가 알아서 골라준다(weighted는
              preference 슬라이더 기준, count_first_density는 trunk 크기 기준). */}
          <p>
            적재 점수 {grade ? `${Math.round(grade.pct)}점` : `${selected.score.toFixed(3)}(등급 계산 불가)`}
            {grade && (
              <span className={styles.grade} data-grade={grade.label}>
                {grade.label}
              </span>
            )}
          </p>
          {grade && (
            <p className={styles.gradeCaption}>
              100점에 가까울수록 가장 좋은 자리, 0점에 가까울수록 가장 나쁜 자리라는 뜻이에요.
              (내부 계산값: {selected.score.toFixed(3)} - 원래 알고리즘은 반대로 낮을수록 좋은 값을 씁니다)
            </p>
          )}
          <button type="button" className={styles.detailToggle} onClick={() => setDetailOpen((v) => !v)}>
            {detailOpen ? "📐 점수 계산식 상세 접기 ▲" : "📐 점수 계산식 상세 보기 ▼"}
          </button>
          {/* score_breakdown.formula: "count_first" 모드는 내부적으로 서로 다른 두 채점
              공식 중 하나를 실제로 쓸 수 있어서(백엔드 algorism_bridge.compute_plan()
              참고), 두 형태를 분기해서 보여준다. */}
          {detailOpen && (selected.score_breakdown.formula === "count_first_density" ? (
            <div className={styles.breakdown}>
              <div className={styles.breakdownRow}><span>높이 항(불리)</span><span>{selected.score_breakdown.height_term.toFixed(3)}</span></div>
              <div className={styles.breakdownRow}><span>새 영역 확장 항(불리)</span><span>{selected.score_breakdown.footprint_growth_term.toFixed(3)}</span></div>
            </div>
          ) : (
            <div className={styles.breakdown}>
              <div className={styles.breakdownRow}><span>높이 항(불리)</span><span>{selected.score_breakdown.height_term.toFixed(3)}</span></div>
              <div className={styles.breakdownRow}><span>접촉면 항(유리)</span><span>-{selected.score_breakdown.contact_term.toFixed(3)}</span></div>
              <div className={styles.breakdownRow}><span>안쪽 벽(A) 항(유리)</span><span>-{selected.score_breakdown.wall_a_term.toFixed(3)}</span></div>
              <div className={styles.breakdownRow}><span>측면 벽(B/C) 항(유리)</span><span>-{selected.score_breakdown.wall_bc_term.toFixed(3)}</span></div>
            </div>
          ))}
        </div>
      ) : (
        <p className={styles.placeholder}>계획 계산 후 박스를 선택하면 상세정보가 표시됩니다</p>
      )}
    </div>
  );
}
