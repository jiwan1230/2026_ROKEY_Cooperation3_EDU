// src/components/RealtimeControlPanel.jsx
// "실시간 제어" 탭 - ControlPanel.jsx(알고리즘 검증 탭)와 같은 계산 파이프라인
// (PlanParamFields/usePlanActions/Scene3DViewer 전부 재사용)을 쓰지만, 박스
// 목록을 무작위 생성/프리셋이 아니라 실제 로봇이 카트 스캔으로 저장한
// all_boxes_corners_*.json 파일을 드롭다운에서 골라 불러온다("그게 UI적으로
// 구현되기 어렵다면 실시간 제어 탭에 저장된 파일을 불러오고 만드는 방식을
// 써보자"는 사용자 피드백). 트렁크는 기존 "트렁크 스캔 파일" 드롭다운
// (state.trunkMaps)을 그대로 재사용한다 - 이미 실제 트렁크 스캔 결과 목록이다.
import { useEffect, useState } from "react";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { usePlanActions } from "../hooks/usePlanActions.js";
import { fetchCartScanFileJson, postParseVisionCorners } from "../api/client.js";
import PlanParamFields from "./PlanParamFields.jsx";
import styles from "./ControlPanel.module.css";

export default function RealtimeControlPanel() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const { handleApprove, handleSend, locked } = usePlanActions();
  const [selectedBoxScanFile, setSelectedBoxScanFile] = useState("");
  const [loadStatus, setLoadStatus] = useState("idle"); // idle | loading

  const setParam = (key, value) => dispatch({ type: "SET_PARAM", payload: { key, value } });

  const loadBoxScanFile = async (filename) => {
    setSelectedBoxScanFile(filename);
    if (!filename) return;
    setLoadStatus("loading");
    try {
      const data = await fetchCartScanFileJson(filename);
      const resp = await postParseVisionCorners(data);
      dispatch({
        type: "LOAD_VISION_BOXES",
        payload: { boxes: resp.boxes, snapshotId: resp.snapshot_id, sourceLabel: `vision:${resp.snapshot_id}` },
      });
    } catch (err) {
      dispatch({
        type: "COMPUTE_ERROR",
        payload: {
          error_code: err.error_code || "BOX_SCAN_FILE_LOAD_FAILED",
          cause: err.cause || err.message || `${filename} 파일을 불러오지 못했습니다.`,
          action: err.action || "파일이 손상되지 않았는지 확인한 뒤 다시 시도하세요.",
        },
      });
    } finally {
      setLoadStatus("idle");
    }
  };

  // 목록이 새로 로드되거나 폴링으로 갱신됐는데 아직 아무 파일도 안 골랐으면
  // 가장 최근 파일(배열 끝 - list_cart_scan_files()가 오래된 순으로 준다)을
  // 자동으로 골라서 불러온다 - 트렁크 맵 드롭다운의 RESOURCES_LOADED 기본
  // 선택과 같은 관례. selectedBoxScanFile을 조건에 넣어 최초 한 번만 동작하고,
  // 사용자가 이후 직접 고른 선택을 덮어쓰지 않는다.
  useEffect(() => {
    if (!selectedBoxScanFile && state.boxScanFiles.length > 0) {
      loadBoxScanFile(state.boxScanFiles[state.boxScanFiles.length - 1]);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [state.boxScanFiles, selectedBoxScanFile]);

  return (
    <div className={styles.panel}>
      <section className={styles.section}>
        <label className={styles.label}>트렁크 스캔 파일</label>
        <select
          className={styles.select}
          value={state.trunkMap}
          disabled={locked}
          onChange={(e) => dispatch({ type: "SET_TRUNK_MAP", payload: e.target.value })}
        >
          {state.trunkMaps.map((name) => <option key={name} value={name}>{name}</option>)}
        </select>

        <label className={styles.label}>카트박스 스캔파일</label>
        <select
          className={styles.select}
          data-testid="box-scan-file-select"
          value={selectedBoxScanFile}
          disabled={locked || loadStatus === "loading"}
          onChange={(e) => loadBoxScanFile(e.target.value)}
        >
          {state.boxScanFiles.length === 0 && <option value="">저장된 스캔 파일 없음</option>}
          {state.boxScanFiles.map((name) => <option key={name} value={name}>{name}</option>)}
        </select>
        {loadStatus === "loading" && <span className={styles.fieldLabel}>불러오는 중...</span>}
        {state.boxSnapshotId && (
          <div className={styles.fieldRow}>
            <span className={styles.fieldLabel}>비전 스냅샷 ID</span>
            <span className={styles.fieldLabel}>{state.boxSnapshotId}</span>
          </div>
        )}
      </section>

      <section className={styles.section}>
        <label className={styles.label}>적재 모드</label>
        <div className={styles.segmented}>
          {[["large_first", "큰 것 우선"], ["count_first", "개수 우선"]].map(([value, text]) => (
            <button
              key={value}
              type="button"
              disabled={locked}
              className={state.params.mode === value ? styles.segmentActive : styles.segment}
              onClick={() => setParam("mode", value)}
            >
              {text}
            </button>
          ))}
        </div>
      </section>

      <PlanParamFields />

      <section className={styles.section}>
        <div className={styles.actions}>
          <button type="button" disabled={state.planState !== "COMPUTED"} onClick={handleApprove}>
            승인
          </button>
          <button type="button" disabled={!(state.planState === "COMPUTED" || state.planState === "APPROVED")}
                  onClick={() => dispatch({ type: "REJECT" })}>
            거부
          </button>
          <button type="button" disabled={state.planState !== "APPROVED"} onClick={handleSend}>
            전송 (MSI2)
          </button>
        </div>
        {state.error && (
          <div className={styles.errorBox}>
            <strong>오류: {state.error.error_code}</strong>
            <p>{state.error.cause}</p>
            <p>권장 조치: {state.error.action}</p>
          </div>
        )}
      </section>
    </div>
  );
}
