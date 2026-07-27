// src/hooks/useResourceLoader.js
import { useEffect } from "react";
import { fetchBoxPresets, fetchCartScanFiles, fetchTrunkMaps } from "../api/client.js";
import { usePlannerDispatch } from "../state/PlannerContext.jsx";

// 트렁크/카트 스캔이 끝나 새 파일이 생겨도, 새로고침 전엔 드롭다운에 안
// 보이던 문제 - 이 주기로 트렁크 맵/카트 스캔파일 목록만 다시 불러온다.
// 박스 프리셋은 스캔 중에 새로 생기는 게 아니라 폴링 대상에서 제외한다.
const TRUNK_MAP_POLL_INTERVAL_MS = 3000;

export function useResourceLoader() {
  const dispatch = usePlannerDispatch();

  useEffect(() => {
    let cancelled = false;

    Promise.all([fetchTrunkMaps(), fetchBoxPresets()])
      .then(([trunkMaps, boxPresets]) => {
        if (cancelled) return;
        dispatch({ type: "RESOURCES_LOADED", payload: { trunkMaps, boxPresets } });
      })
      .catch((err) => {
        if (cancelled) return;
        dispatch({
          type: "COMPUTE_ERROR",
          payload: {
            error_code: err.error_code || "RESOURCE_LOAD_FAILED",
            cause: err.cause || err.message,
            action: "백엔드(localhost:5000)가 실행 중인지 확인한 뒤 페이지를 새로고침하세요.",
          },
        });
      });

    // "실시간 제어" 탭의 카트박스 스캔파일 드롭다운용 - 실패해도(백엔드가
    // 아직 없는 옛 버전이거나 일시적 오류) "알고리즘 검증" 탭 쪽 흐름을
    // 막으면 안 되므로 조용히 콘솔에만 남긴다.
    fetchCartScanFiles()
      .then((boxScanFiles) => {
        if (cancelled) return;
        dispatch({ type: "BOX_SCAN_FILES_REFRESHED", payload: { boxScanFiles } });
      })
      .catch((err) => {
        if (cancelled) return;
        console.warn("카트 스캔파일 목록 조회 실패:", err);
      });

    const intervalId = setInterval(() => {
      fetchTrunkMaps()
        .then((trunkMaps) => {
          if (cancelled) return;
          dispatch({ type: "TRUNK_MAPS_REFRESHED", payload: { trunkMaps } });
        })
        // 폴링 실패는 화면을 깨뜨릴 만한 일이 아니라(스캔 대기 중 백엔드가
        // 잠깐 안 뜬 것일 수도 있음) COMPUTE_ERROR로 사용자에게 띄우지 않고
        // 콘솔에만 남긴다 - 다음 주기에 다시 시도된다.
        .catch((err) => {
          if (cancelled) return;
          console.warn("트렁크 맵 목록 폴링 실패:", err);
        });
      fetchCartScanFiles()
        .then((boxScanFiles) => {
          if (cancelled) return;
          dispatch({ type: "BOX_SCAN_FILES_REFRESHED", payload: { boxScanFiles } });
        })
        .catch((err) => {
          if (cancelled) return;
          console.warn("카트 스캔파일 목록 폴링 실패:", err);
        });
    }, TRUNK_MAP_POLL_INTERVAL_MS);

    return () => {
      cancelled = true;
      clearInterval(intervalId);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
}
