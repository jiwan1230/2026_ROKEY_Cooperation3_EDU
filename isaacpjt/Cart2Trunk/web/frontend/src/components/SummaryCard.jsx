// src/components/SummaryCard.jsx
import { useState } from "react";
import { usePlannerState } from "../state/PlannerContext.jsx";
import { gradeBoxScore, gradeUtilization, labelForPct } from "../utils/scoreGrading.js";
import styles from "./SummaryCard.module.css";

const STATUS_LABEL = {
  NOT_COMPUTED: "① 파라미터를 입력하면 자동으로 계산됩니다",
  COMPUTING: "계산 중...",
  COMPUTED: "계산됨 - 승인하거나 파라미터를 조정하세요",
  APPROVED: "승인됨 - 파라미터 잠김 - MSI2로 전송할 수 있습니다",
};

export default function SummaryCard() {
  const state = usePlannerState();
  const summary = state.result?.summary;
  const placed = state.result?.placed || [];
  // 원점수/등급 계산 근거/등급 기준표는 알고리즘 내부 계산을 설명하는
  // 세부 정보라 - 배지(우수/양호 등)만 봐도 충분한 사람이 대부분이라는
  // 피드백으로, 기본은 접어두고 필요할 때만 펼친다(ControlPanel의 "고급
  // 설정"과 같은 패턴).
  const [advancedOpen, setAdvancedOpen] = useState(false);
  // 평균 점수 등급은 모든 박스가 "같은" 채점 공식을 썼을 때만 의미가 있다 -
  // count_first 모드는 내부적으로 weighted/count_first_density 두 공식 중
  // 하나를 박스마다 다르게 쓸 수 있는데, 두 공식이 섞이면 스케일이 완전히
  // 달라(예: 섭씨/화씨를 같이 평균내는 것과 같음) 평균 자체가 무의미해진다.
  // 반면 "개수 우선 모드에서도 평균 점수/등급을 보고 싶다"는 피드백대로,
  // 전부 같은 공식(weighted만, 또는 count_first_density만)이면 그 공식
  // 기준으로 등급을 매길 수 있으므로 formula가 균일한 경우까지 확장한다.
  const uniformFormula = placed.length > 0 && placed.every(
    (p) => p.score_breakdown.formula === placed[0].score_breakdown.formula,
  ) ? placed[0].score_breakdown.formula : null;
  const scoreGrade = uniformFormula
    ? gradeBoxScore(uniformFormula, summary?.avg_score ?? 0, {
        preferences: state.params,
        trunk: state.result?.trunk,
      })
    : null;

  // "종합 점수" = 완주율(몇 개나 실었는지) × 배치 품질(평균 점수를 등급 계산과
  // 같은 0~100 스케일로 환산한 값) - 원점수(낮을수록 좋음, 마이너스일 수 있음)를
  // 그대로 보여주면 "왜 마이너스냐"는 혼란이 생긴다는 피드백으로, 첫눈에 보이는
  // 숫자는 항상 0~100 양수로 통일한다.
  const completionRate = summary && summary.total > 0 ? summary.placed / summary.total : 0;
  const overallPct = scoreGrade ? completionRate * scoreGrade.pct : null;
  const overallGrade = overallPct != null ? labelForPct(overallPct) : null;

  return (
    <div className={styles.card}>
      {summary && (
        <div className={`${styles.row} ${styles.overallRow}`}>
          <span>종합 점수</span>
          <strong>
            {overallPct != null ? `${overallPct.toFixed(0)}점` : "-"}
            {overallGrade && (
              <span className={styles.grade} data-grade={overallGrade}>{overallGrade}</span>
            )}
          </strong>
        </div>
      )}
      {summary && overallPct != null && (
        <div className={styles.criteria}>
          적재율 {summary.placed}/{summary.total}({(completionRate * 100).toFixed(0)}%)
          × 배치 품질 {scoreGrade.pct.toFixed(0)}점
        </div>
      )}

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
        <span>평균 점수</span>
        <strong>
          {summary ? (scoreGrade ? `${scoreGrade.pct.toFixed(0)}점` : "-") : "-"}
          {scoreGrade && (
            <span className={styles.grade} data-grade={scoreGrade.label}>{scoreGrade.label}</span>
          )}
        </strong>
      </div>
      {summary && (
        <button
          type="button"
          className={styles.advancedToggle}
          data-testid="score-advanced-toggle"
          onClick={() => setAdvancedOpen((open) => !open)}
        >
          점수·등급 계산 기준 {advancedOpen ? "숨기기 ▴" : "자세히 보기 ▾"}
        </button>
      )}
      {advancedOpen && summary && (
        <div className={styles.criteria}>
          원점수 <span>{summary.avg_score.toFixed(3)}</span>(낮을수록 좋은 자리 → 등급으로 환산)
        </div>
      )}
      {advancedOpen && scoreGrade && (
        <div className={styles.criteria}>
          점수 등급: {uniformFormula === "weighted" ? "지금 우선순위 설정" : "지금 트렁크 크기"} 기준
          최선~최악 범위 중 상위 <span>{scoreGrade.pct.toFixed(0)}%</span>
        </div>
      )}
      <div className={styles.row}><span>계산 시간</span><strong>{summary ? `${summary.calc_time_ms.toFixed(0)}ms` : "-"}</strong></div>
      <div className={styles.status}>상태: {STATUS_LABEL[state.planState]}</div>
      {advancedOpen && summary && (
        <div className={styles.criteria}>
          <div className={styles.criteriaLabel}>공간 활용률 등급 기준</div>
          <ul className={styles.thresholdList}>
            <li><span>우수</span><span>22% 이상</span></li>
            <li><span>양호</span><span>14% ~ 22%</span></li>
            <li><span>보통</span><span>8% ~ 14%</span></li>
            <li><span>개선 필요</span><span>8% 미만</span></li>
          </ul>
          <div className={styles.criteriaNote}>
            박스를 전부 실어도 트렁크 규모상 활용률이 높게 나오기 어려움(실측 기준)
          </div>
        </div>
      )}
    </div>
  );
}
