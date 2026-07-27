// src/components/VerdictBanner.jsx
// "이 알고리즘을 처음 쓰는 고객도 점수만 보고 뭘 해야 하는지 알아야 한다"는
// 피드백 - SummaryCard의 여러 줄짜리 숫자표 대신, 종합 점수 하나를 큰 문장
// 결론(+ 실행 버튼)으로 먼저 보여준다. gradeOverallScore(완주율×배치품질)를
// 그대로 재사용하므로 숫자 자체는 SummaryCard의 "종합 점수"와 항상 같다.
import { usePlannerState } from "../state/PlannerContext.jsx";
import { gradeOverallScore } from "../utils/scoreGrading.js";
import styles from "./VerdictBanner.module.css";

const TONE_BY_LABEL = {
  "우수": { icon: "✅", headline: "이 적재 방식은 아주 좋습니다" },
  "양호": { icon: "👍", headline: "괜찮은 적재 방식입니다" },
  "보통": { icon: "🤔", headline: "다시 검토해보면 더 좋아질 수 있어요" },
  "개선 필요": { icon: "⚠️", headline: "이 적재 방식은 개선이 필요합니다" },
};

function buildReason(placedCount, totalCount, unplacedCount, qualityPct) {
  const loadedPart = unplacedCount > 0
    ? `박스 ${placedCount}/${totalCount}개를 실었어요(${unplacedCount}개는 못 실었어요).`
    : `박스 ${placedCount}개를 전부 실었어요.`;
  const qualityPart = qualityPct >= 80
    ? "실린 자리들도 전부 최적이에요."
    : qualityPct >= 50
      ? "실린 자리들도 대체로 좋아요."
      : "실린 자리 중 일부는 아쉬운 위치예요.";
  return `${loadedPart} ${qualityPart}`;
}

export default function VerdictBanner({ onGoToRobotTab }) {
  const state = usePlannerState();
  const summary = state.result?.summary;
  const placed = state.result?.placed || [];
  // App.module.css의 resultArea는 3행 grid(배너/topRow/BoxDetailPanel)라,
  // 여기서 null을 반환하면(계산 전 등) grid 아이템 수가 줄어서 다음 두
  // 컴포넌트가 한 칸씩 밀려 올라간다(3행 grid에 자식이 2개만 있게 됨) -
  // 항상 같은 자리(1행)를 차지하되 내용이 없을 땐 스타일 없는 빈 div로
  // 높이를 0에 가깝게 유지한다.
  if (!summary) return <div />;

  const overallGrade = gradeOverallScore(placed, summary.total, {
    preferences: state.params,
    trunk: state.result?.trunk,
  });
  if (!overallGrade) return <div />;

  const tone = TONE_BY_LABEL[overallGrade.label];
  const pct = Math.round(overallGrade.pct);

  return (
    <div className={styles.banner} data-grade={overallGrade.label}>
      <div className={styles.headlineRow}>
        <span className={styles.icon}>{tone.icon}</span>
        <span className={styles.headline}>{tone.headline}</span>
        <span className={styles.score}>{pct}점</span>
      </div>
      <p className={styles.reason}>
        {overallGrade.completionRate === 0
          ? "박스를 하나도 싣지 못했어요 - 왼쪽 설정을 조정해보세요."
          : buildReason(placed.length, summary.total, summary.unplaced, overallGrade.qualityPct)}
      </p>
      {overallGrade.completionRate > 0 && (
        <button type="button" className={styles.cta} onClick={onGoToRobotTab}>
          🤖 이 순서로 로봇에게 적재 시작하기
        </button>
      )}
    </div>
  );
}
