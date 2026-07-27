# cart2trunk_bridge ROS2 워크스페이스

Cart2Trunk의 "UI PC ↔ Isaac Sim PC" ROS2 통신 첫 단계(트렁크 스캔) 구현.
`/cart2trunk/trunk_scan` 액션으로 89.trunk_scan_holonomic.py ->
90.export_trunk_map_holonomic.py 파이프라인을 원격에서 실행하고, 결과 PLY
포인트클라우드를 청크로 나눠 전송한다. 자세한 설계 배경은
`/home/rokey/.claude/plans/harmonic-sprouting-stallman.md` 참고.

원본 파이프라인 스크립트(89.py/90.py)는 여기서 서브프로세스로만 호출하며
절대 수정하지 않는다.

## 구조

```
src/
├── cart2trunk_interfaces/   # 커스텀 액션 정의(TrunkScan.action)
└── cart2trunk_bridge/
    ├── trunk_scan_action_server.py   # Isaac Sim PC에서 실행
    ├── trunk_scan_action_client.py   # UI PC에서 실행(Flask가 서브프로세스로 호출)
    └── pipeline_runner.py            # 89/90 서브프로세스 실행 + PLY float32 변환
```

## 빌드 (최초 1회, 또는 코드 변경 시)

```bash
cd isaacpjt/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 한 대에서 검증하기

터미널 3개:

```bash
# 1) Isaac Sim PC 역할 - 액션 서버
source /opt/ros/humble/setup.bash
source isaacpjt/ros2_ws/install/setup.bash
ros2 run cart2trunk_bridge trunk_scan_action_server \
  --ros-args --params-file isaacpjt/ros2_ws/src/cart2trunk_bridge/config/trunk_scan_server.params.yaml

# 2) UI 백엔드
cd isaacpjt/Cart2Trunk/web/backend && source venv/bin/activate && python app.py

# 3) 프론트엔드
cd isaacpjt/Cart2Trunk/web/frontend && npm run dev
```

웹 UI에서 "트렁크 스캔" 버튼을 누르면: Flask(`routes/robot.py`) ->
`robot_bridge.py`가 `ros2 run cart2trunk_bridge trunk_scan_client`를 서브프로세스로
실행 -> 액션 서버가 89.py+90.py 실행 -> PLY를 32KB 청크로 전송 -> 클라이언트가
재조립해서 `web/backend/received_scans/`에 저장 -> Flask가 `url` 필드로 응답 ->
프론트엔드가 그 PLY를 받아 실제 점군으로 렌더링.

CLI로만 빠르게 확인하고 싶다면(웹 없이):
```bash
ros2 action send_goal /cart2trunk/trunk_scan cart2trunk_interfaces/action/TrunkScan "{headless: true}" --feedback
```

## 청크 크기(chunk_size_bytes) - 왜 32768(32KiB)인가

처음엔 256KiB로 설계했는데, 실측(로컬호스트 rmw_fastrtps_cpp)에서 feedback
메시지가 대략 165KB~240KB 구간일 때 **간헐적으로 청크 전체가 유실**되는 문제를
발견했다(재현: 32~150KB는 매번 성공, 165KB 이상은 실패 빈발 - UDP 프래그먼트
유실로 추정, DDS RELIABLE QoS인데도 발생). 여러 크기로 반복 테스트한 결과
32768바이트는 안정적이어서 기본값으로 채택했다. 이후 스캔 결과가 훨씬 커지면
(예: 필터링을 덜 하는 경우) 이 값을 더 낮춰야 할 수 있다 -
`config/trunk_scan_server.params.yaml`의 `chunk_size_bytes` 파라미터로 조정.

## 2대 배포 시 추가로 필요한 것

- **양쪽 머신 `ROS_DOMAIN_ID`를 동일하게 설정**한다(이 프로젝트에 지금까지
  전혀 명시된 적이 없었음 - 배포 전에 값 하나를 정해서 양쪽 `.bashrc` 등에
  넣어둘 것, 예: `export ROS_DOMAIN_ID=42`).
- `ROS_LOCALHOST_ONLY`가 설정돼 있다면 0으로 하거나 아예 unset한다(설정돼
  있으면 다른 머신을 아예 못 찾는다).
- 두 머신 사이에 DDS 디스커버리/데이터용 UDP 트래픽(멀티캐스트 포함)이
  방화벽에 막히지 않아야 한다.
- `config/trunk_scan_server.params.yaml`의 `cart2trunk_dir`/`isaac_python_sh`/
  `ros2_bridge_ld_library_path`는 **Isaac Sim PC 기준 절대경로**다. 머신마다
  홈 디렉터리가 다를 수 있으므로(예: 이 저장소에는 이미 서로 다른 개발자의
  홈 경로가 하드코딩된 파일이 있다) 해당 머신에 맞게 이 파일을 고치거나
  `--ros-args -p <이름>:=<값>`으로 덮어써서 실행한다.
- UI PC의 `web/backend/robot_bridge.py`는 `ros2_ws`가 `Cart2Trunk`의 형제
  디렉터리(`isaacpjt/ros2_ws`)라고 가정한다 - 전체 `isaacpjt/` 폴더를 통째로
  똑같이 복사/클론하면 자동으로 맞는다. 다른 위치에 두는 경우
  `CART2TRUNK_ROS2_WS_SETUP` 환경변수로 `install/setup.bash` 경로를 직접
  지정할 수 있다.

## 범위 밖(다음 단계)

- 카트 스캔(`/api/robot/cart-scan`), 픽앤플레이스(`/api/robot/pick-and-place`)는
  아직 더미다 - 이번과 같은 방식(액션 서버/클라이언트 노드 + Flask 어댑터)으로
  이어서 노드화할 것.
- `/cart2trunk/trunk_map`(적재 알고리즘용 요약 데이터, `TRUNK_MAP_ROS2_HANDOFF.md`
  참고)은 이 워크스페이스와 별개로 진행 중인 다른 작업이다 - 혼동하지 말 것.
