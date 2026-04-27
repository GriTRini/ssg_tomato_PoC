import numpy as np
import numpy.typing as npt
from typing import Optional, List
import datetime
from . import rt_bind as _rtb

def create_robot(model_name: str = "m1013"):
    """
    C++ RobotFactory를 호출하여 모델에 맞는 로봇 객체를 생성합니다.
    """
    return _rtb.create_robot(model_name)

class Robot(_rtb.RobotBase):
    """
    Doosan Robotics 실로봇 제어를 위한 파이썬 인터페이스 (Header 역할).
    C++ DsrRobot 클래스의 기능을 상속받아 파이썬 친화적인 API를 제공합니다.
    """

    def __init__(self, model_name: str = "m1013", update_dt_ms: int = 1):
        """
        로봇 객체를 초기화합니다.
        
        Args:
            model_name: 로봇 모델 이름 (기본값: "m1013")
            update_dt_ms: 제어 루프 주기 [ms] (기본값: 1ms)
        """
        # C++ RobotBase(model_name, update_dt) 생성자 호출
        super().__init__(model_name, update_dt_ms)

    # ==============================================================
    # 🔌 연결 및 하드웨어 제어 API
    # ==============================================================

    def open_connection(self, ip: str = "192.168.1.30", port: int = 12345) -> bool:
        """로봇 컨트롤러와 TCP/IP 연결을 시도합니다."""
        return super().open_connection(ip, port)

    def connect_rt(self, ip: str = "192.168.1.30", port: int = 12347) -> bool:
        """실시간 제어를 위한 UDP RT 연결을 수행합니다."""
        return super().connect_rt(ip, port)

    def servo_on(self) -> bool:
        """로봇의 서보를 켭니다. 궤적 엔진이 자동 초기화됩니다."""
        return super().servo_on()

    def servo_off(self) -> bool:
        """실시간 제어 루프를 중단하고 서보를 끕니다."""
        return super().servo_off()

    def set_digital_output(self, index: int, value: bool):
        """컨트롤 박스의 디지털 출력을 제어합니다."""
        super().set_digital_output(index, value)

    # ==============================================================
    # 🔧 도구(Tool) 파라미터 및 안전 설정 API (🌟 신규 추가)
    # ==============================================================
    
    def add_tool(self, name: str, weight: float, cog: List[float], inertia: List[float]) -> bool:
        """
        새로운 툴(도구)의 스펙을 컨트롤러에 등록합니다.
        
        Args:
            name: 툴 이름 (예: "Gripper_A")
            weight: 툴의 무게 (kg)
            cog: 무게중심 [X, Y, Z] (mm)
            inertia: 관성모멘트 [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        """
        return super().add_tool(name, weight, cog, inertia)

    def set_tool(self, name: str) -> bool:
        """
        미리 등록된 툴을 활성화하여 동역학 계산(Dynamics)에 즉시 반영합니다.
        """
        return super().set_tool(name)

    def del_tool(self, name: str) -> bool:
        """등록된 툴을 제어기 목록에서 삭제합니다."""
        return super().del_tool(name)

    def change_collision_sensitivity(self, sensitivity: float) -> bool:
        """충돌 감지 민감도를 설정합니다. (일반적으로 1~100 범위 사용)"""
        return super().change_collision_sensitivity(sensitivity)

    # ==============================================================
    # 🚨 모니터링 및 에러 처리 API (🌟 신규 추가)
    # ==============================================================
    
    def pop_alarm(self) -> Optional[_rtb.RobotAlarm]:
        """
        큐에 쌓여있는 로봇 알람(Error)을 하나 꺼내옵니다.
        에러가 없다면 None을 반환하므로 Non-blocking으로 폴링하기 좋습니다.
        
        Returns:
            RobotAlarm 객체 (time, level, group, index, param 튜플 포함) 또는 None
        """
        return super().pop_alarm()

    # ==============================================================
    # 🚀 궤적 제어 API (Movement Commands)
    # ==============================================================

    def set_tcp(self, x: float, y: float, z: float, r: float, p: float, yaw: float):
        """툴 오프셋(TCP)을 설정합니다. (단위: m, deg)"""
        super().set_tcp(x, y, z, r, p, yaw)

    def trapj(self, goal_q: npt.NDArray) -> bool:
        """관절 공간 사다리꼴 궤적 이동 명령 (Non-blocking)"""
        return super().trapj(goal_q)

    def attrl(self, *args, **kwargs) -> bool:
        """
        작업 공간 어트랙터 기반 이동 명령 (🌟 오버로딩 지원하도록 수정)
        
        사용법 1 (Matrix): robot.attrl(goal_tmat, kp=50.0)
        사용법 2 (Euler) : robot.attrl(x, y, z, r_deg, p_deg, yaw_deg, kp=50.0)
        """
        return super().attrl(*args, **kwargs)

    def align_to_floor(self, yaw_deg: float = 0.0, kp: float = 100.0) -> bool:
        """현재 위치를 유지하며 TCP를 바닥 방향(-Z)으로 정렬합니다."""
        return super().align_to_floor(yaw_deg, kp)

    def align_to_front(self, kp: float = 100.0) -> bool:
        """현재 위치를 유지하며 TCP를 정면 방향으로 정렬합니다. (🌟 신규 추가)"""
        return super().align_to_front(kp)

    def stop(self):
        """로봇을 즉시 부드럽게 정지시킵니다."""
        super().stop()

    def solve_forward(self, q: npt.NDArray) -> npt.NDArray[np.float64]:
        """
        입력한 관절 각도(q)에 대한 순기구학(Forward Kinematics) 행렬을 계산하여 반환합니다. (🌟 신규 추가)
        """
        return super().solve_forward(q)

    def goal_reached(self, q_th: float = 2.0, p_th: float = 0.002, r_th: float = 3.0) -> bool:
        """로봇이 목표 지점에 도달했는지 확인합니다."""
        return super().goal_reached(q_th, p_th, r_th)

    # ==============================================================
    # 📊 상태 모니터링 API (Properties)
    # ==============================================================

    @property
    def angles(self) -> npt.NDArray[np.float64]:
        """실시간 로봇 관절 각도 [deg]"""
        return super().angles

    @property
    def angvels(self) -> npt.NDArray[np.float64]:
        """실시간 로봇 관절 각속도 [deg/s] (🌟 신규 추가)"""
        return super().angvels

    @property
    def tmat(self) -> npt.NDArray[np.float64]:
        """실시간 TCP 포즈 행렬 (4x4)"""
        return super().tmat

    @property
    def flange_tmat(self) -> npt.NDArray[np.float64]:
        """TCP 오프셋이 적용되지 않은 순수 손목(Flange) 포즈 행렬 (4x4)"""
        return super().flange_tmat

    @property
    def jmat(self) -> npt.NDArray[np.float64]:
        """현재 자세의 자코비안 행렬 (6x6) (🌟 신규 추가)"""
        return super().jmat

    @property
    def task_vel(self) -> npt.NDArray[np.float64]:
        """현재 작업 공간(Task space) 속도 (6x1) (🌟 신규 추가)"""
        return super().task_vel

    @property
    def pos_enorm(self) -> float:
        """현재 목표 위치와 실제 위치 사이의 거리 오차 [m]"""
        # 참고: pos_enorm은 C++ RobotBase에 바인딩되어 있지 않다면 별도 구현이 필요할 수 있습니다. 
        # C++ 코드에 포함되어 있다면 정상 작동합니다.
        return super().pos_enorm