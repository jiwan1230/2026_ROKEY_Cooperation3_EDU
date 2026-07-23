"""
run_scan_once_accumulate.py
run_scan_once.py의 "저비용 검증"용 변형 - "여러 프레임을 각각 검출하고 개수로
투표"하는 대신, "여러 프레임의 raw point cloud를 먼저 다 합친 뒤 딱 한 번만
전처리+RANSAC 검출"한다.

배경 (42.crate_scan_setup_table_box_verify.py와 짝)
----
지금까지(run_scan_once.py)는 프레임마다 매번 전체 검출(RANSAC/DBSCAN)을 새로
돌리고, Open3D의 segment_plane()이 시드 고정 없는 RANSAC이라 프레임마다 결과가
달라지는 걸 "여러 프레임 관찰 후 최빈값(개수) 채택"으로 다뤄왔다 - 이건 이미 계산된
결과들 중에서 "그나마 제일 흔한 것"을 고르는 방식이라, 애초에 입력 자체가 부실하면
(포인트가 적고 노이즈가 크면) 최빈값도 부실할 수 있다.

이 스크립트는 다른 전략을 시도한다: 카메라를 전혀 안 움직인 채(정합/좌표 변환 문제가
없음) 같은 자리에서 DURATION 동안 관찰되는 모든 프레임의 raw point(depth_to_points()
직후, 전처리 전)를 그냥 다 쌓아서 하나의 훨씬 조밀한 point cloud를 만들고, 그걸
딱 한 번만 전처리(다운샘플+아웃라이어 제거)하고 딱 한 번만 RANSAC/DBSCAN 검출을
돌린다 - "여러 결과 중 대표를 고르기"가 아니라 "더 나은 입력 하나로 한 번에 풀기".

검출 로직(RANSAC/DBSCAN/받침 매칭/8꼭짓점 복원) 자체는 box_top_extractor.py의
DepthTopmostBoxExtractor.process_scene_cloud()를 그대로 재사용한다 - 원래
depth_callback 안에 있던 코드를 그대로 옮긴 것뿐이라(로직 변경 없음), 이 스크립트가
따로 구현한 검출 코드는 하나도 없다.

사용법은 run_scan_once.py와 동일:
    DISPLAY=:1 python3 run_scan_once_accumulate.py --marker /tmp/scan_table.done
"""
import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from std_msgs.msg import Header

from box_top_extractor import DepthTopmostBoxExtractor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0,
                         help="raw point cloud를 누적할 시간(초) - run_scan_once.py와 같은 기본값")
    parser.add_argument("--marker", required=True,
                         help="저장 완료를 알리는 마커 파일 경로 (Isaac Sim 스크립트가 이 파일 존재를 폴링함)")
    args = parser.parse_args()

    marker_path = Path(args.marker)
    marker_path.parent.mkdir(parents=True, exist_ok=True)
    if marker_path.exists():
        marker_path.unlink()

    rclpy.init()
    node = DepthTopmostBoxExtractor()
    try:
        end_time = time.time() + args.duration
        raw_frames = []
        frames_observed = 0
        while time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(node.latest_raw_scene_points) > 0:
                raw_frames.append(node.latest_raw_scene_points)
                frames_observed += 1

        if not raw_frames:
            print("[run_scan_once_accumulate] 경고: 관찰 시간 동안 point cloud를 한 번도 못 받음 - 저장 생략")
            node.save_current_cloud()
        else:
            merged_raw_points = np.vstack(raw_frames)
            print(f"[run_scan_once_accumulate] {frames_observed}프레임 누적, "
                  f"합친 raw point 수={len(merged_raw_points)} "
                  f"(프레임당 평균 {len(merged_raw_points) // frames_observed}개)")

            merged_pcd = node.preprocess_cloud(merged_raw_points)
            processed_count = len(np.asarray(merged_pcd.points))
            print(f"[run_scan_once_accumulate] 전처리(다운샘플+아웃라이어 제거) 후 "
                  f"point 수={processed_count}")

            header = node.latest_header
            if header is None:
                # 실제 depth 프레임을 한 번도 못 받았는데 raw_frames가 채워질 수는
                # 없지만(latest_raw_scene_points가 depth_callback에서만 채워짐),
                # 방어적으로 빈 Header를 만들어둔다 - 퍼블리시 토픽의 frame_id/시간
                # 정보만 비게 될 뿐, save_current_cloud()의 실제 저장 결과에는 영향 없다.
                header = Header()

            candidates = node.process_scene_cloud(merged_pcd, header)
            print(f"[run_scan_once_accumulate] 검출 후보={len(candidates)}, "
                  f"최종 복원 박스={len(node.latest_boxes)}")
            node.save_current_cloud()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    marker_path.write_text("done")
    print(f"[run_scan_once_accumulate] 마커 생성: {marker_path}")


if __name__ == "__main__":
    main()
