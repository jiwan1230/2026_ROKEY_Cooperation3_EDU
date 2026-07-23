"""
run_scan_once_frame_survey.py
run_scan_once.py의 진단 전용 변형 - run_scan_once.py 자체는 전혀 손대지 않는다
(35/38/39/40/41/42/43.py 전부가 그 스크립트를 터미널 B 기본값으로 참조하므로,
거기 진단 로직을 넣으면 전부 같이 영향받는다 - 그래서 별도 스크립트로 분리).

목적: 적층 시나리오에서 "Small이 일부 프레임에서만 검출되는가"를 직접 확인하기
위해, DURATION 동안 관찰한 모든 프레임의 박스 개수와 개수 분포까지는
run_scan_once.py와 동일하게 계산하되, **최빈값과 다른 개수가 나온 프레임들의
박스별 상세 정보(footprint/지지 타입/base_link 기준 높이)를 전부 출력**한다 -
run_scan_once.py는 최빈값 프레임 하나만 저장하고 나머지는 버리기 때문에 이
정보를 볼 방법이 없었다.

동작(마커 핸드셰이크, 최빈값 프레임 저장)은 run_scan_once.py와 완전히 동일하다 -
_dedup_boxes/DEDUP_RADIUS_M도 그대로 import해서 재사용(로직 중복 없음). 그 위에
프레임별 상세 기록 + 비최빈값 프레임 출력만 추가된 것.

사용법은 run_scan_once.py와 동일:
    DISPLAY=:1 python3 run_scan_once_frame_survey.py --marker /tmp/scan_table.done
"""
import argparse
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy

from box_top_extractor import DepthTopmostBoxExtractor
from run_scan_once import DEDUP_RADIUS_M, _dedup_boxes  # noqa: F401 (재사용, 중복 구현 없음)


def _describe_box(node, box):
    """박스 하나를 (footprint, 높이, 지지 타입, base_link 기준 중심 z)로 요약한다.
    save_current_cloud()가 최종 저장 시 쓰는 것과 같은 camera->base_link 변환
    (self._base_R/self._base_t)을 재사용해서, 서로 다른 후보의 높이를 같은 기준으로
    비교할 수 있게 한다."""
    corners_cam = np.asarray(box["corners"], dtype=np.float64)
    corners_base = corners_cam @ node._base_R.T + node._base_t
    top = box["top"]
    base_z_center = float(corners_base[:, 2].mean())
    return (
        f"support={box['support_type']:>8} "
        f"footprint={top.width:.3f}x{top.height:.3f} "
        f"base_link_z={base_z_center:.3f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--marker", required=True)
    args = parser.parse_args()

    marker_path = Path(args.marker)
    marker_path.parent.mkdir(parents=True, exist_ok=True)
    if marker_path.exists():
        marker_path.unlink()

    rclpy.init()
    node = DepthTopmostBoxExtractor()
    try:
        end_time = time.time() + args.duration
        snapshots = []  # [(frame_idx, dedup_count, deduped_boxes), ...]
        frame_idx = 0
        while time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest_boxes:
                deduped = _dedup_boxes(node.latest_boxes)
                snapshots.append((frame_idx, len(deduped), deduped))
                frame_idx += 1

        if not snapshots:
            print("[frame_survey] 경고: 관찰 시간 동안 박스가 한 번도 감지되지 않음 - 저장 생략")
            node.save_current_cloud()
        else:
            counts = [c for _, c, _ in snapshots]
            mode_count, mode_freq = Counter(counts).most_common(1)[0]
            print(f"[frame_survey] {len(snapshots)}프레임 관찰, 개수 분포={dict(Counter(counts))}, "
                  f"최빈값={mode_count}개({mode_freq}/{len(snapshots)}프레임)")

            print(f"\n[frame_survey] 최빈값({mode_count})과 다른 개수가 나온 프레임 상세 --------")
            _any_off_mode = False
            for _fidx, _count, _boxes in snapshots:
                if _count == mode_count:
                    continue
                _any_off_mode = True
                print(f"  프레임#{_fidx} (개수={_count}):")
                for _b in _boxes:
                    print(f"    box_id={_b['box_id']} {_describe_box(node, _b)}")
            if not _any_off_mode:
                print("  (모든 프레임이 최빈값과 같은 개수 - 비교할 프레임 없음)")
            print("----------------------------------------------------------------\n")

            print(f"[frame_survey] 참고: 최빈값({mode_count}) 프레임 하나의 상세도 출력 --------")
            _mode_example = next(boxes for _, c, boxes in snapshots if c == mode_count)
            for _b in _mode_example:
                print(f"    box_id={_b['box_id']} {_describe_box(node, _b)}")
            print("----------------------------------------------------------------\n")

            best = next(boxes for _, c, boxes in reversed(snapshots) if c == mode_count)
            node.latest_boxes = best
            node.save_current_cloud()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    marker_path.write_text("done")
    print(f"[frame_survey] 마커 생성: {marker_path}")


if __name__ == "__main__":
    main()
