"""
01_object3d_schema.py
① Planner 입력 정의 (Object3D 입력 스키마)
================================================
상태: 🟢 확정 (Q1~Q4 팀 답변 반영, 7/20)

[비유]
택배 상자에 붙은 송장과 같다. 택배기사가 상자를 열어보지 않아도
송장(받는 사람/크기/무게)만 보고 처리할 수 있는 것처럼, Planner(선욱)는
Object3D라는 "정보표 한 장"만 받으면 박스를 직접 안 봐도 계산할 수 있다.

[확정된 필드] (팀 문서에 이미 명시됨, 논쟁 여지 없음)
    id, center_xyz, size_xyz, volume

[Q1~Q4 팀 답변 반영 완료]

Q1 (준형 - Vision): confidence 0.7 이하는 Vision 단에서 자동 필터링됨.
    confidence 필드 자체는 Object3D에 포함해서 넘겨주기로 함.

Q2 (지완 - 좌표계) ✅ 완전히 해소됨: 트렁크도, 카트도, 박스(Object3D)도
    전부 M0609(로봇팔) base 좌표계 하나로 통일해서 만든다고 확인함
    (스케치 참고: 로봇이 원점이고, 트렁크 점들(빨강)과 카트 점들(초록)이
    전부 그 원점 기준 벡터로 표현됨). 별도의 "트렁크 로컬 좌표계"나
    "월드 절대좌표"로의 변환 자체가 필요 없음 - 처음부터 끝까지 같은
    좌표계 하나만 쓰면 됨.
    ⚠️ 다만 우리 알고리즘(③ Extreme Point) 내부적으로는 "박스가 놓일
    자리를 (0,0,0)부터 계산"하는 방식이라, 트렁크 데이터가 로봇 원점
    기준값(예: (-0.3, 0.5, 0.1) 같은 음수 포함 값)으로 오면, 이걸 계산용
    로컬 좌표로 한 번 평행이동하는 게 필요함. 근데 이건 "팀이 해줘야 하는
    변환"이 아니라 "우리가 받은 데이터로 내부에서 알아서 처리하는 것" -
    ②에서 처리함.

Q3 (봉투): 두 가지 답변이 있었음 -
    - 지완(운영 관점): 봉투는 박스처럼 "쌓지" 않음. 박스를 먼저 다 쌓고,
      봉투는 남는 공간에 얹거나 공간이 없으면 바닥에 내려놓는 별도 방식.
    - 준형(인식 관점): 종이 bag은 박스와 비슷한 스키마(AABB)로 표현
      가능하지만, YOLO가 박스와 혼동할 위험 있어 추가 학습 필요.
    → 진행 중 (팀에서 계속 논의 중). 지금 알고리즘은 계속 박스 전용으로 진행.

Q4 (준형 - 박스 ID 지속성): 재스캔해도 같은 박스가 같은 id를 유지하는지
    물어봤더니, "지금은 보장 안 되지만 나중에 해결될 예정"이라는 답변.
    → 지금 당장 우리 쪽에서 급하게 처리할 필요는 없음. 다만 ⑨(재스캔 후
    재계획)를 실제로 완성하려면 결국 필요한 전제조건이라, "나중에"가 언제
    쯤인지는 1차 통합(7/22) 전에 한 번 더 확인하는 게 좋음. 지금은 ⑨를
    "매번 트렁크를 처음부터 새로 인식한다"는 보수적 가정으로 임시 구현.
"""

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Object3D:
    """
    Vision(준형) → Planner(선욱) 로 넘어오는 물품 정보표 한 장.

    좌표계: center_xyz는 M0609(로봇팔) base 좌표계 기준 (Q2 확정).
    트렁크(TrunkWorldMap)도 카트도 전부 같은 좌표계를 쓰므로 별도 변환 불필요.
    """
    id: str
    center_xyz: Tuple[float, float, float]   # M0609 base 좌표계 기준
    size_xyz: Tuple[float, float, float]     # (width, depth, height) 순서
    volume: float
    confidence: float                         # Q1 확정: 0.7 초과 값만 들어옴

    # Q4 반영: 재스캔 전후 유지되는 진짜 지속 ID. 지금은 보장 안 됨 (추후 해결 예정).
    # None이면 아직 추적 로직이 없다는 뜻 (현재 상태).
    object_id: Optional[str] = None

    # 봉투 관련 필드는 Q3 논의가 끝날 때까지 사용하지 않음 (박스 전용으로 진행)


def object3d_to_box(obj: Object3D):
    """
    Object3D(Vision 출력)를 03~08번 파일에서 쓰는 Box로 변환하는 어댑터.
    좌표계가 이미 통일돼 있으므로(Q2), 크기(size_xyz)만 그대로 옮기면 됨.
    위치(center_xyz)는 트렁크 로컬 오프셋 계산 후 ⑦에서 다시 계산하므로
    여기서는 다루지 않음.
    """
    from importlib import import_module
    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    Box = import_module("03_extreme_point_candidates").Box

    w, d, h = obj.size_xyz
    return Box(id=obj.id, width=w, depth=d, height=h)


# ---------------------------------------------------------------------------
# 지금까지 검증에 사용한 임시 테스트 데이터 (기획안 9.6절 박스 3종 규격)
# ---------------------------------------------------------------------------

TEST_OBJECT3D = {
    "SMALL": Object3D("B1", (0.0, 0.0, 0.0), (0.30, 0.20, 0.15), 0.30 * 0.20 * 0.15, confidence=0.95),
    "MEDIUM": Object3D("B2", (0.0, 0.0, 0.0), (0.40, 0.30, 0.25), 0.40 * 0.30 * 0.25, confidence=0.95),
    "LARGE": Object3D("B3", (0.0, 0.0, 0.0), (0.50, 0.35, 0.30), 0.50 * 0.35 * 0.30, confidence=0.95),
}