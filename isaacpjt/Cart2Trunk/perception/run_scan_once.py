"""
run_scan_once.py
box_top_extractor.py의 DepthTopmostBoxExtractor를 인터랙티브 cv2 키 입력('s' 키) 없이
한 번 실행해서 save_current_cloud()까지 자동으로 트리거하는 헤드리스 드라이버.

배경
----
box_top_extractor.py는 원래 사람이 cv2 창을 보면서 박스가 잘 잡혔을 때 's'를 눌러
저장 시점을 정하는 라이브 GUI 도구다. 35.crate_scan_setup.py가 로봇을 스캔 자세로
수렴시키고 ROS2 depth 토픽을 publish하는 동안, 이 스크립트를 별도 프로세스로 띄워서
"일정 시간 depth 프레임을 받은 뒤 자동 저장"으로 트리거만 바꾼다 - 검출 로직
(depth_callback, RANSAC/DBSCAN, 8꼭짓점 복원)은 원본 클래스를 import만 해서 전혀
손대지 않고 그대로 재사용한다.

cv2.imshow가 depth_callback 안에서 무조건 호출되므로(Qt 백엔드, Xvfb/오프스크린
플러그인 없음 확인됨) 실제 X 디스플레이가 있는 상태로 실행해야 한다:
    DISPLAY=:1 python3 run_scan_once.py --marker /tmp/scan_table.done

--marker로 지정한 경로에, save_current_cloud() 호출 직후 빈 파일을 하나 만든다 -
35.crate_scan_setup.py는 이 파일이 생길 때까지 world.step()을 반복하며 기다렸다가
다음 단계(포즈 전환 또는 씬 저장)로 넘어간다. 타이밍을 손으로 맞추는 대신 파일
존재 여부로 핸드셰이크하는 것 - 렌더링 속도에 따라 실제 대기 시간이 달라져도 안전하다.

[왜 "마지막 프레임을 그냥 저장"이 아니라 "여러 프레임을 관찰해서 대표값을 고르는가"]
처음엔 "일정 시간 기다렸다가 그 순간의 마지막 프레임을 그냥 저장"했었다 - 원본
인터랙티브 도구('s' 키)와 검출 로직(RANSAC/DBSCAN)은 완전히 같으니 결과도 같을
거라 가정했는데, 실제로는 아니었다(실측: box 3이 중복으로 잡힌 저장본이 나온
사례). Open3D의 segment_plane()이 내부적으로 시드 고정 없는 RANSAC이라, 정지된
장면을 반복 스캔해도 프레임마다 검출된 박스 개수가 다르게 나올 수 있다 - 실제로
"연속 N프레임 동일할 때까지 대기"를 먼저 시도했지만 10초(수십 프레임) 동안 단
한 번도 연속 2프레임이 같은 개수로 나오지 않았다(실측 확인) - 그만큼 프레임 간
편차가 크다는 뜻이라 "안정화를 기다린다"는 전제 자체가 이 노이즈에는 안 맞았다.
사람이 's'를 누를 때는 화면을 보면서 "지금까지 본 것 중 제일 그럴듯한" 순간을
암묵적으로 고르는 것에 가깝다 - 이걸 흉내내려고, DURATION 동안 관찰한 모든
프레임의 (중복 제거 후) 박스 개수 중 최빈값(mode)을 "대표 개수"로 삼고, 그
개수와 일치하는 프레임들 중 하나(가장 마지막 것)를 그대로 저장한다.

[프레임 내 중복 후보 제거]
"연속 안정화 대기"와 별개로, 한 프레임 안에서도 같은 물리적 박스가 두 후보로
겹쳐 잡히는 경우가 있다(경계선 근처에서 RANSAC이 평면을 두 조각으로 쪼개는 것으로
추정). 프레임을 기록할 때마다 후보들의 중심(8꼭짓점 평균, 카메라 좌표계) 사이
거리가 DEDUP_RADIUS_M보다 가까우면 같은 물리적 박스로 보고 Point 수가 더 많은
쪽만 남긴다 - 이후 "박스 개수" 비교/최빈값 계산도 이 중복 제거된 개수를 기준으로
한다.
"""
import argparse
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy

from box_top_extractor import DepthTopmostBoxExtractor

# 같은 물리적 박스가 후보 2개로 겹쳐 잡혔다고 볼 중심 간 거리(m, 카메라 좌표계).
# 테이블 위 실제 박스 간 최소 간격(TABLE_BOXES 배치, ~0.3m 이상)보다 훨씬 작게 잡아서
# "진짜 서로 다른 두 박스"를 중복으로 오판하지 않게 한다.
DEDUP_RADIUS_M = 0.07


def _dedup_boxes(boxes):
    """중심이 가까운 후보 중 completed_points가 더 많은 쪽만 남긴다."""
    kept = []
    kept_centers = []
    for box in sorted(boxes, key=lambda b: -len(b["completed_points"])):
        center = np.mean(np.asarray(box["corners"], dtype=np.float64), axis=0)
        if any(np.linalg.norm(center - c) < DEDUP_RADIUS_M for c in kept_centers):
            continue
        kept.append(box)
        kept_centers.append(center)
    return kept


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0,
                         help="여러 프레임을 관찰하며 대기할 시간(초) - 이 안에서 최빈값 프레임을 고른다")
    parser.add_argument("--marker", required=True,
                         help="저장 완료를 알리는 마커 파일 경로 (35.py가 이 파일 존재를 폴링함)")
    args = parser.parse_args()

    marker_path = Path(args.marker)
    marker_path.parent.mkdir(parents=True, exist_ok=True)
    if marker_path.exists():
        marker_path.unlink()

    rclpy.init()
    node = DepthTopmostBoxExtractor()
    try:
        end_time = time.time() + args.duration
        snapshots = []  # [(dedup_count, deduped_boxes), ...] 관찰 순서대로
        while time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest_boxes:
                deduped = _dedup_boxes(node.latest_boxes)
                snapshots.append((len(deduped), deduped))

        if not snapshots:
            print("[run_scan_once] 경고: 관찰 시간 동안 박스가 한 번도 감지되지 않음 - 저장 생략")
            node.save_current_cloud()
        else:
            counts = [c for c, _ in snapshots]
            mode_count, mode_freq = Counter(counts).most_common(1)[0]
            # 최빈값과 일치하는 마지막 스냅샷을 채택(가장 최근 상태를 우선).
            best = next(boxes for c, boxes in reversed(snapshots) if c == mode_count)
            print(f"[run_scan_once] {len(snapshots)}프레임 관찰, 개수 분포={dict(Counter(counts))}, "
                  f"최빈값={mode_count}개({mode_freq}/{len(snapshots)}프레임) - 이 개수의 마지막 프레임을 저장")
            node.latest_boxes = best
            node.save_current_cloud()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    marker_path.write_text("done")
    print(f"[run_scan_once] 마커 생성: {marker_path}")


if __name__ == "__main__":
    main()
