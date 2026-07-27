// src/components/PickPlacePanel.jsx
// Pick&Place 칸 - 진행현황 바 + 현재 진행 작업 텍스트 + 상태(Run/Stop/
// Warning).
//
// [2026-07-28] postPickAndPlace()가 실제 pick_and_place ROS2 Action(4박스 기준
// 실측 15~20분)을 호출하도록 백엔드가 바뀌었는데, 예전 이 컴포넌트는 그 응답을
// 완전히 무시(.catch(() => {}))하고 프론트엔드 자체 타이머(PICK_PLACE_STEPS를
// 700ms 간격으로 순회)로 몇 초 만에 가짜 "완료"를 보여주고 있었다 - 지금은
// postPickAndPlace()의 실제 완료/실패를 그대로 기다렸다가 반영한다.
//
// [2026-07-28 후속] 박스 단위 실시간 진행 상황도 연결했다 - pick_and_place_client가
// Feedback을 파일에 계속 append하고(robot_bridge.py), GET
// /api/robot/pick-and-place/progress를 2초 간격으로 폴링해서 새로 생긴
// 이벤트(box_started/box_done)만 관제 로그에 추가하고 진행률/진행 작업
// 텍스트도 실제 박스 인덱스 기준으로 갱신한다.
import { useEffect, useRef, useState } from "react";
import { fetchPickAndPlaceProgress, postPickAndPlace } from "../api/client.js";
import styles from "./PickPlacePanel.module.css";

const POLL_INTERVAL_MS = 2000;

function describeEvent(ev) {
  const boxLabel = ev.box_count ? `박스 ${ev.box_index + 1}/${ev.box_count}` : null;
  switch (ev.stage) {
    case "started":
      return "픽앤플레이스 시작됨(Isaac Sim)";
    case "box_started":
      return `${boxLabel} 시작 (id=${ev.box_id})`;
    case "box_done":
      return `${boxLabel} 배치 완료`;
    default:
      return null; // "done"/"error"는 handleStart()의 최종 완료/실패 로그와 중복이라 건너뜀
  }
}

function progressFromEvents(events) {
  // 가장 최근의 box_started/box_done 이벤트를 찾아서 대략적인 %를 계산한다 -
  // box_done이면 그 박스까지 끝난 것으로, box_started면 절반 진행한 것으로 본다.
  for (let i = events.length - 1; i >= 0; i--) {
    const ev = events[i];
    if (ev.stage === "box_started" || ev.stage === "box_done") {
      if (!ev.box_count) return { pct: 0, label: null };
      const done = ev.box_index + (ev.stage === "box_done" ? 1 : 0.5);
      return {
        pct: Math.round((done / ev.box_count) * 100),
        label: ev.stage === "box_done"
          ? `박스 ${ev.box_index + 1}/${ev.box_count} 배치 완료`
          : `박스 ${ev.box_index + 1}/${ev.box_count} 진행 중 (id=${ev.box_id})`,
      };
    }
  }
  return { pct: 0, label: null };
}

export default function PickPlacePanel({ onLog = () => {} }) {
  const [runState, setRunState] = useState("stopped"); // "stopped" | "running"
  const [resultText, setResultText] = useState(null); // 완료/실패 후 보여줄 텍스트
  const [isError, setIsError] = useState(false);
  const [liveProgress, setLiveProgress] = useState({ pct: 0, label: null });

  const seenEventCountRef = useRef(0);
  const pollTimerRef = useRef(null);
  const onLogRef = useRef(onLog);
  onLogRef.current = onLog;

  const stopPolling = () => {
    if (pollTimerRef.current) {
      clearInterval(pollTimerRef.current);
      pollTimerRef.current = null;
    }
  };

  const startPolling = () => {
    seenEventCountRef.current = 0;
    setLiveProgress({ pct: 0, label: null });
    pollTimerRef.current = setInterval(async () => {
      let events;
      try {
        ({ events } = await fetchPickAndPlaceProgress());
      } catch {
        return; // 폴링 한 번 실패는 무시 - 다음 tick에서 다시 시도
      }
      const newEvents = events.slice(seenEventCountRef.current);
      seenEventCountRef.current = events.length;
      for (const ev of newEvents) {
        const text = describeEvent(ev);
        if (text) onLogRef.current(text);
      }
      if (newEvents.length > 0) setLiveProgress(progressFromEvents(events));
    }, POLL_INTERVAL_MS);
  };

  useEffect(() => stopPolling, []); // 언마운트 시 인터벌 정리

  const handleStart = async () => {
    setRunState("running");
    setResultText(null);
    setIsError(false);
    onLog("픽앤플레이스 시작");
    startPolling();

    try {
      const resp = await postPickAndPlace();
      setResultText(
        resp.boxes_total != null
          ? `완료 (${resp.boxes_placed}/${resp.boxes_total}개 배치)`
          : "완료"
      );
      onLog("픽앤플레이스 완료");
    } catch (err) {
      setIsError(true);
      setResultText(err.message || "실패");
      onLog(`픽앤플레이스 실패: ${err.message || ""}`);
    } finally {
      stopPolling();
      setRunState("stopped");
    }
  };

  const badgeStatus = runState === "running" ? "running" : isError ? "warning" : "stopped";
  const pct = runState === "running" ? liveProgress.pct : resultText ? 100 : 0;

  return (
    <div className={styles.panel}>
      <span className={styles.title}>Pick&amp;Place</span>

      <div className={styles.progressSection}>
        <label className={styles.label}>진행현황</label>
        <div className={styles.progressBar}>
          <div className={styles.progressFill} style={{ width: `${pct}%` }} />
        </div>
        <span className={styles.progressPct}>{pct}%</span>
      </div>

      <div className={styles.taskSection}>
        <label className={styles.label}>현재 진행 작업</label>
        <div className={styles.taskText} data-testid="current-task">
          {runState === "running"
            ? liveProgress.label ?? "실행 중 - 박스 수에 따라 최대 20분 정도 걸릴 수 있습니다"
            : resultText ?? "대기 중"}
        </div>
      </div>

      <div className={styles.statusSection}>
        <label className={styles.label}>현재 상태</label>
        <span className={styles.statusBadge} data-status={badgeStatus} data-testid="pick-place-status">
          {runState === "running" ? "Run" : isError ? "Warning" : "Stop"}
        </span>
      </div>

      <button type="button" data-testid="trigger-pickAndPlace" disabled={runState === "running"} onClick={handleStart}>
        픽앤플레이스 시작
      </button>
    </div>
  );
}
