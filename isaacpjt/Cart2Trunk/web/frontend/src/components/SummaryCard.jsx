// src/components/SummaryCard.jsx
import { useState } from "react";
import { usePlannerState } from "../state/PlannerContext.jsx";
import { gradeBoxScore, gradeUtilization, labelForPct } from "../utils/scoreGrading.js";
import { SCENARIOS } from "./scenarioTheme.js";
import styles from "./SummaryCard.module.css";

export default function SummaryCard({ activeScenarioId }) {
  const state = usePlannerState();
  const activeScenario = SCENARIOS.find((s) => s.id === activeScenarioId);
  const summary = state.result?.summary;
  const placed = state.result?.placed || [];
  // 원점수/등급 계산 근거/등급 기준표는 알고리즘 내부 계산을 설명하는
  // 세부 정보라 - 배지(우수/양호 등)만 봐도 충분한 사람이 대부분이라는
  // 피드백으로, 기본은 접어두고 필요할 때만 펼친다(ControlPanel의 "고급
  // 설정"과 같은 패턴).
  const [advancedOpen, setAdvancedOpen] = useState(false);
  // "배치 품질"(예전엔 "평균 점수"라는 이름이었으나 종합 점수 캡션의
  // "배치 품질"과 같은 값인데 이름만 달라 헷갈린다는 피드백으로 워딩을
  // 통일함, 2026-07-28) 등급은 모든 박스가 "같은" 채점 공식을 썼을 때만
  // 의미가 있다 - count_first 모드는 내부적으로 weighted/count_first_density
  // 두 공식 중 하나를 박스마다 다르게 쓸 수 있는데, 두 공식이 섞이면
  // 스케일이 완전히 달라(예: 섭씨/화씨를 같이 평균내는 것과 같음) 평균
  // 자체가 무의미해진다. 반면 "개수 우선 모드에서도 배치 품질/등급을 보고
  // 싶다"는 피드백대로, 전부 같은 공식(weighted만, 또는 count_first_density만)
  // 이면 그 공식 기준으로 등급을 매길 수 있으므로 formula가 균일한 경우까지
  // 확장한다.
  const uniformFormula = placed.length > 0 && placed.every(
    (p) => p.score_breakdown.formula === placed[0].score_breakdown.formula,
  ) ? placed[0].score_breakdown.formula : null;
  const scoreGrade = uniformFormula
    ? gradeBoxScore(uniformFormula, summary?.avg_score ?? 0, {
        preferences: state.params,
        trunk: state.result?.trunk,
      })
    : null;

  // "종합 점수" = 완주율(몇 개나 실었는지) × 배치 품질(박스들 원점수 평균을
  // 등급 계산과 같은 0~100 스케일로 환산한 값) - 원점수(낮을수록 좋음, 마이너스일 수 있음)를
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

      {/* 전체/적재됨/미적재는 배지가 안 붙는 단순 숫자라(잘릴 위험이 없음)
          공간 절약을 위해 라벨+값을 한 줄에 놓는 compact 행을 쓴다 - 1920x1080
          같은 흔한 해상도에서도 스크롤 없이 한 화면에 다 들어와야 한다는
          피드백(브라우저 자체 chrome 만큼 실제 페이지 높이가 줄어드는 경우도
          있어 여유를 넉넉히 둠). */}
      <div className={styles.rowCompact}><span>전체</span><strong>{summary ? summary.total : "-"}</strong></div>
      <div className={styles.rowCompact}><span>적재됨</span><strong>{summary ? summary.placed : "-"}</strong></div>
      <div className={styles.rowCompact}><span>미적재</span><strong>{summary ? summary.unplaced : "-"}</strong></div>
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
        <span>배치 품질</span>
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
      {advancedOpen && scoreGrade && (
        <div className={styles.criteria}>
          {/* "상위 N%"는 석차 표현(작을수록 좋음)이라 - 100에 가까울수록
              좋은 이 값(최악=0점, 최선=100점)과 방향이 반대라 헷갈린다는
              피드백으로 "상위" 표현을 빼고 "몇 점 위치"로 바꿨다
              (2026-07-28). */}
          배치 품질: {uniformFormula === "weighted" ? "지금 우선순위 설정" : "지금 트렁크 크기"} 기준
          최악을 0점, 최선을 100점으로 봤을 때 <span>{scoreGrade.pct.toFixed(0)}점</span>
        </div>
      )}
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
      {activeScenario && (
        <div className={styles.scenarioNote} data-testid="scenario-note">
          <div className={styles.scenarioNoteTitle}>📋 {activeScenario.label} 시나리오</div>
          <div>{activeScenario.description}</div>
        </div>
      )}
    </div>
  );
}
