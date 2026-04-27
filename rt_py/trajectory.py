import numpy as np
import numpy.typing as npt
from typing import Optional
import rt_bind as _rtb

class RobotModel(_rtb.RobotModel):
    """로봇 기구학 모델 정보를 관리하는 클래스"""
    def __init__(self, model_name: str):
        super().__init__(model_name)

class TrajGenerator(_rtb.TrajGenerator):
    """
    순수 시뮬레이션용 궤적 생성기 인터페이스.
    하드웨어 연결 없이 파이썬에서 update(dt)를 직접 호출하여 궤적을 연산합니다.
    """
    
    def __init__(self):
        super().__init__()

    def initialize(self, model: RobotModel, q: npt.NDArray, dq: npt.NDArray, ddq: npt.NDArray):
        """궤적 엔진을 특정 상태로 초기화합니다."""
        super().initialize(model, q, dq, ddq)

    def update(self, dt: float):
        """시간 간격(dt)만큼 궤적 상태를 갱신합니다."""
        super().update(dt)

    def set_tcp(self, x: float, y: float, z: float, r: float, p: float, yaw: float):
        super().set_tcp(x, y, z, r, p, yaw)

    def trapj(self, goal_q: npt.NDArray, goal_dq: Optional[npt.NDArray] = None) -> bool:
        return super().trapj(goal_q, goal_dq)

    def attrl(self, goal_tmat: npt.NDArray, kp: float = 50.0) -> bool:
        return super().attrl(goal_tmat, kp)

    def align_to_floor(self, yaw_deg: float = 0.0, kp: float = 100.0) -> bool:
        return super().align_to_floor(yaw_deg, kp)

    def goal_reached(self, q_th: float = 2.0, p_th: float = 0.002, r_th: float = 3.0) -> bool:
        return super().goal_reached(q_th, p_th, r_th)

    @property
    def angles(self) -> npt.NDArray: return super().angles

    @property
    def tmat(self) -> npt.NDArray: return super().tmat

    @property
    def flange_tmat(self) -> npt.NDArray:
        """TCP 오프셋이 적용되지 않은 순수 손목 포즈"""
        return super().flange_tmat