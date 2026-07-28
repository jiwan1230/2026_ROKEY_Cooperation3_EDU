// src/components/TabBar.jsx
// 스캐닝/플래닝/픽앤플레이스/알고리즘 검증 탭 전환 - react-router 없이
// App.jsx의 로컬 상태로 전환하기 위한 순수 프레젠테이션 컴포넌트. 실제
// 작업 흐름(스캔 -> 계획 -> 적재) 순서를 그대로 탭 순서로 반영하고,
// "알고리즘 검증"은 개발/검증용이라 마지막으로 뺐다.
import styles from "./TabBar.module.css";

const TABS = [
  { key: "scan", label: "스캐닝" },
  { key: "realtime", label: "플래닝" },
  { key: "pickplace", label: "픽앤플레이스" },
  { key: "verify", label: "알고리즘 검증" },
];

export default function TabBar({ activeTab, onSelect }) {
  return (
    <nav className={styles.tabBar}>
      {TABS.map(({ key, label }) => (
        <button
          key={key}
          type="button"
          data-testid={`tab-${key}`}
          aria-current={activeTab === key ? "page" : undefined}
          className={activeTab === key ? styles.tabActive : styles.tab}
          onClick={() => onSelect(key)}
        >
          {label}
        </button>
      ))}
    </nav>
  );
}
