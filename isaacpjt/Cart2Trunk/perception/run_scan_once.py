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
"""
import argparse
import time
from pathlib import Path

import rclpy

from box_top_extractor import DepthTopmostBoxExtractor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=6.0,
                         help="depth 프레임을 받으며 대기할 시간(초) - 단일 프레임 기반 검출이라 몇 초면 충분")
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
        frames_seen = 0
        while time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest_boxes:
                frames_seen += 1
        print(f"[run_scan_once] 대기 종료 - 마지막 프레임 박스 수={len(node.latest_boxes)}, "
              f"박스가 감지된 프레임 수(누적, 참고용)={frames_seen}")
        node.save_current_cloud()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    marker_path.write_text("done")
    print(f"[run_scan_once] 마커 생성: {marker_path}")


if __name__ == "__main__":
    main()
