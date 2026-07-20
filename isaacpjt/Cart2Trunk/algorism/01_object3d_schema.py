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
    → Planner 입장에서는 "confidence는 항상 0.7 초과인 값만 들어온다"고
      가정해도 되지만, 필드 자체는 존재하므로 로깅/디버깅용으로 활용 가능.

Q2 (지완 - 좌표계): center_xyz는 월드 절대좌표가 아니라
    "M0609(로봇팔) base 좌표계" 기준. Nova Carter(AMR) 위에 로봇팔이 있어서
    AMR이 이동하면 좌표계 자체도 같이 움직이지만, 카트가 항상 동일한
    상대 위치에 놓인다는 전제로 그 시점의 M0609 base 기준 좌표를 사용.
    ⚠️ 여전히 미정: M0609 base 좌표계 → 트렁크 로컬 좌표계 변환을
    누가/언제 하는지는 아직 확정 안 됨 (지완에게 재질의 필요).

Q3 (봉투): 두 가지 답변이 있었음 -
    - 지완(운영 관점): 봉투는 박스처럼 "쌓지" 않음. 박스를 먼저 다 쌓고,
      봉투는 남는 공간에 얹거나 공간이 없으면 바닥에 내려놓는 별도 방식.
    - 준형(인식 관점): 종이 bag은 박스와 비슷한 스키마(AABB)로 표현
      가능하지만, 종이 bag과 박스가 형태가 비슷해서 YOLO가 같은 클래스로
      혼동할 위험이 있음 - 추가 학습 필요, 아직 인식 신뢰도 불확실.
    → 결론: 지금 알고리즘은 박스 전용으로 계속 진행. 봉투는 (1) 적재
      전략(쌓지 않음)과 (2) 인식 신뢰도(YOLO 혼동 위험) 두 가지가 모두
      해결돼야 안전하게 투입 가능 - 아직 범위 밖.

Q4 (준형 - 박스 ID 지속성, 신규 질문): 재스캔해도 같은 박스가 같은 id를
    유지하는지 물어봤더니, "YOLO 검출만으로는 보장 안 됨"이라는 답변.
    지속적인 ID가 필요하면 이전 Object3D 목록과 새 검출 결과를 매칭하는
    객체 추적/상태 관리 과정이 별도로 필요함. 준형 제안: Object3D에
    object_id 필드를 추가하고, 중앙 관리 노드가 위치·크기·작업 이력을
    바탕으로 기존 ID를 유지하는 방식.
    ⚠️ 중요: 이건 Planner(우리) 책임 범위 밖의 별도 컴포넌트가 필요하다는
    뜻. ⑨(재스캔 후 재계획)이 "박스 ID가 유지된다"고 가정하고 있었는데,
    이 가정이 지금은 성립하지 않음 - 누가 이 추적 로직을 만들지 팀
    차원에서 정해야 함 (준형? 지완의 중앙 관리 노드?).
"""

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Object3D:
    """
    Vision(준형) → Planner(선욱) 로 넘어오는 물품 정보표 한 장.

    좌표계 주의: center_xyz는 M0609(로봇팔) base 좌표계 기준 (Q2 확정,
    트렁크 로컬 좌표계로의 변환 주체는 아직 미정).
    """
    id: str
    center_xyz: Tuple[float, float, float]   # M0609 base 좌표계 기준 (Q2 확정)
    size_xyz: Tuple[float, float, float]     # (width, depth, height) 순서
    volume: float
    confidence: float                         # Q1 확정: 0.7 초과 값만 들어옴 (필드는 항상 포함)

    # ⚠️ Q4 반영: object_id는 "재스캔 전후로도 유지되는" 진짜 지속 ID.
    # 위의 id는 그냥 이번 스캔에서의 검출 순번일 수 있어 재스캔하면 바뀔 수
    # 있음. object_id가 None이면 아직 추적 로직이 없다는 뜻 (지금 상태).
    object_id: Optional[str] = None

    # 봉투 관련 필드는 Q3 답변에 따라 지금 범위에서는 사용하지 않음
    # (적재 전략 + 인식 신뢰도 둘 다 미해결 - 박스 전용으로 진행)


def object3d_to_box(obj: Object3D):
    """
    Object3D(Vision 출력)를 03~08번 파일에서 쓰는 Box로 변환하는 어댑터.

    ⚠️ TODO: center_xyz가 M0609 base 좌표계라서, 트렁크 로컬 좌표계로 쓰려면
    변환 로직이 필요함 (Q2 후속 - 변환 주체 미정).
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