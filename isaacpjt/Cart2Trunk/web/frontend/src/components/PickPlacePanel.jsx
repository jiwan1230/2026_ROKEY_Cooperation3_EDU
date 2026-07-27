// src/components/PickPlacePanel.jsx
// Pick&Place 칸 - 진행현황 바 + 현재 진행 작업 텍스트 + 상태(Run/Stop/
// Warning).
//
// [2026-07-28] postPickAndPlace()가 실제 pick_and_place ROS2 Action(4박스 기준
// 실측 15~20분)을 호출하도록 백엔드가 바뀌었는데, 예전 이 컴포넌트는 그 응답을
// 완전히 무시(.catch(() => {}))하고 프론트엔드 자체 타이머(PICK_PLACE_STEPS를
// 700ms 간격으로 순회)로 몇 초 만에 가짜 "완료"를 보여주고 있었다 - 실제로는
// 아직 로봇이 몇 분째 움직이고 있는데 화면은 이미 끝났다고 나오는 상태였다.
// 지금은 postPickAndPlace()의 실제 완료/실패를 그대로 기다렸다가 반영한다.
// 박스 단위 실시간 진행률(%)은 아직 웹 레이어까지 안 뚫려있어서(ROS2
// /isaac_task_runner/status 토픽 -> 웹 API 연동은 다음 단계) 정확한 %를 보여줄
// 수 없다 - 진행 중에는 막연한 "진행 중" 표시만 하고, 끝나면 실제 boxes_placed/
// boxes_total로 결과를 보여준다.
import { useState } from "react";
import { postPickAndPlace } from "../api/client.js";
import styles from "./PickPlacePanel.module.css";

export default function PickPlacePanel({ onLog = () => {} }) {
  const [runState, setRunState] = useState("stopped"); // "stopped" | "running"
  const [resultText, setResultText] = useState(null); // 완료/실패 후 보여줄 텍스트
  const [isError, setIsError] = useState(false);

  const handleStart = async () => {
    setRunState("running");
    setResultText(null);
    setIsError(false);
    onLog("픽앤플레이스 시작");

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
      setRunState("stopped");
    }
  };

  const badgeStatus = runState === "running" ? "running" : isError ? "warning" : "stopped";

  return (
    <div className={styles.panel}>
      <span className={styles.title}>Pick&amp;Place</span>

      <div className={styles.progressSection}>
        <label className={styles.label}>진행현황</label>
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: runState === "running" || resultText ? "100%" : "0%" }}
          />
        </div>
        <span className={styles.progressPct}>
          {runState === "running" ? "진행 중" : resultText ? "100%" : "0%"}
        </span>
      </div>

      <div className={styles.taskSection}>
        <label className={styles.label}>현재 진행 작업</label>
        <div className={styles.taskText} data-testid="current-task">
          {runState === "running"
            ? "실행 중 - 박스 수에 따라 최대 20분 정도 걸릴 수 있습니다"
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
