"""
01_object3d_schema.py
① Planner 입력 정의 (Object3D 입력 스키마)
================================================
상태: 🟡 초안 완료 — 팀 확인 질문 3개 답변 대기

[비유]
택배 상자에 붙은 송장과 같다. 택배기사가 상자를 열어보지 않아도
송장(받는 사람/크기/무게)만 보고 처리할 수 있는 것처럼, Planner(선욱)는
Object3D라는 "정보표 한 장"만 받으면 박스를 직접 안 봐도 계산할 수 있다.

[확정된 필드] (팀 문서에 이미 명시됨, 논쟁 여지 없음)
    id, center_xyz, size_xyz, volume

[팀에 확인해야 할 질문 3개]
    Q1 (준형 - Vision) : Object3D에 confidence 필드가 포함되나요,
        아니면 Vision이 이미 필터링 끝낸 값만 넘어오나요?
    Q2 (지완 - Isaac Sim 환경) : center_xyz가 월드 절대좌표인가요,
        트렁크 기준 상대좌표인가요? 절대좌표라면 변환은 누가 하나요?
    Q3 (준형+지완 합의) : 봉투도 박스와 같은 스키마(AABB)로 표현하고,
        shape_type: "bag" + stackable: false만 추가하는 방향으로 가도 될까요?

질문 답변이 오기 전까지는 아래 스키마가 "최종 확정"이 아니라 "잠정안"이다.
"""

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Object3D:
    """
    Vision(준형) → Planner(선욱) 로 넘어오는 물품 정보표 한 장.

    ⚠️ TODO (팀 답변 도착 시):
        - Q1 답 오면 confidence 필드 유지/제거 확정
        - Q2 답 오면 좌표계 주석 확정 (지금은 "미정"으로 표기)
        - Q3 답 오면 shape_type/stackable 필드를 봉투에도 적용할지 확정
    """
    id: str
    center_xyz: Tuple[float, float, float]   # ⚠️ Q2: 절대좌표/상대좌표 미정
    size_xyz: Tuple[float, float, float]     # (width, depth, height) 순서
    volume: float

    # ---- 잠정 필드 (팀 답변 대기) ----
    shape_type: str = "box"          # "box" | "bag" — ⚠️ Q3 답 대기
    stackable: bool = True           # ⚠️ Q3 답 대기
    confidence: Optional[float] = None  # ⚠️ Q1 답 대기 (None이면 필터링 이미 끝난 것으로 간주)


def object3d_to_box(obj: Object3D):
    """
    Object3D(Vision 출력)를 03~08번 파일에서 쓰는 Box로 변환하는 어댑터.
    좌표계(Q2)가 확정되기 전까지는 center_xyz를 그대로 사용 - 검증 시 주의.
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
    "SMALL": Object3D("B1", (0.0, 0.0, 0.0), (0.30, 0.20, 0.15), 0.30 * 0.20 * 0.15),
    "MEDIUM": Object3D("B2", (0.0, 0.0, 0.0), (0.40, 0.30, 0.25), 0.40 * 0.30 * 0.25),
    "LARGE": Object3D("B3", (0.0, 0.0, 0.0), (0.50, 0.35, 0.30), 0.50 * 0.35 * 0.30),
}
