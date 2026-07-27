// src/components/SummaryCard.jsx
import { usePlannerState } from "../state/PlannerContext.jsx";
import { gradeOverallScore, gradeUtilization } from "../utils/scoreGrading.js";
import { SCENARIOS } from "./scenarioTheme.js";
import styles from "./SummaryCard.module.css";

const STATUS_LABEL = {
  NOT_COMPUTED: "① 파라미터를 입력하면 자동으로 계산됩니다",
  COMPUTING: "계산 중...",
  COMPUTED: "계산됨 - 승인하거나 파라미터를 조정하세요",
  APPROVED: "승인됨 - 파라미터 잠김 - MSI2로 전송할 수 있습니다",
};

export default function SummaryCard({ activeScenarioId }) {
  const state = usePlannerState();
  const activeScenario = SCENARIOS.find((s) => s.id === activeScenarioId);
  const summary = state.result?.summary;
  const placed = state.result?.placed || [];
  // "적재 점수"가 실린 박스들의 자리 품질만 보고 미적재 개수를 안 본다는
  // 피드백 - 완주율(적재됨/전체) × 배치 품질(자리 품질 0~100)을 곱해서
  // "종합 점수"로 보여준다. 다 실었어도 자리가 나쁘면, 자리가 좋아도 다 못
  // 실었으면 둘 다 이 점수가 깎인다(gradeOverallScore 주석 참고).
  const overallGrade = gradeOverallScore(placed, summary?.total ?? 0, {
    preferences: state.params,
    trunk: state.result?.trunk,
  });

  return (
    <div className={styles.card}>
      <div className={styles.row}><span>전체</span><strong>{summary ? summary.total : "-"}</strong></div>
      <div className={styles.row}><span>적재됨</span><strong>{summary ? summary.placed : "-"}</strong></div>
      <div className={styles.row}><span>미적재</span><strong>{summary ? summary.unplaced : "-"}</strong></div>
      <div className={styles.row}>
        <span>공간 활용률</span>
        <strong>
          {summary ? `${summary.utilization_pct.toFixed(1)}%` : "-"}
          {summary && (
            <span className={styles.grade} data-grade={gradeUtilization(summary.utilization_pct).label}>
              {gradeUtilization(summary.utilization_pct).label}
            </span>
          )}
        </strong>
      </div>
      <div className={styles.row}>
        <span>종합 점수</span>
        <strong>
          {overallGrade ? `${Math.round(overallGrade.pct)}점` : "-"}
          {overallGrade && (
            <span className={styles.grade} data-grade={overallGrade.label}>{overallGrade.label}</span>
          )}
        </strong>
      </div>
      {overallGrade && (
        <div className={styles.criteria}>
          {overallGrade.completionRate === 0 ? (
            "📦 박스를 하나도 싣지 못해 0점입니다"
          ) : (
            <>
              📦 종합 점수 = 적재율({placed.length}/{summary.total}={Math.round(overallGrade.completionRate * 100)}%) × 배치 품질({Math.round(overallGrade.qualityPct)}점)
              - 다 실었는지와 자리가 좋았는지를 함께 봅니다
            </>
          )}
        </div>
      )}
      <div className={styles.row}><span>계산 시간</span><strong>{summary ? `${summary.calc_time_ms.toFixed(0)}ms` : "-"}</strong></div>
      <div className={styles.status}>상태: {STATUS_LABEL[state.planState]}</div>
      {summary && (
        <div className={styles.criteria}>
          공간 활용률 기준: 22%↑ 우수 · 14~22% 양호 · 8~14% 보통 · 8%↓ 개선 필요
        </div>
      )}
      {activeScenario && (
        <div className={styles.scenarioNote}>
          <div className={styles.scenarioNoteTitle}>📋 {activeScenario.label} 시나리오</div>
          <div>{activeScenario.description}</div>
        </div>
      )}
    </div>
  );
}
