import numpy as np
import numpy.typing as npt
from typing import Optional, List
import datetime
import rt_bind as _rtb

def create_robot(model_name: str = "m1013"):
    """
    C++ RobotFactory를 호출하여 모델에 맞는 로봇 객체를 생성합니다.
    """
    return _rtb.create_robot(model_name)


class Robot:
    """
    Doosan(DSR) 및 Hanwha(HCR) 로봇 통합 제어를 위한 파이썬 인터페이스.
    C++의 RobotBase 기반 인스턴스들을 내부적으로 래핑하여 단일 공통 API를 제공합니다.
    """

    # 🌟 [중요] pybind11 상속(super()) 구조를 걷어내고 일반 파이썬 클래스로 정의합니다.
    def __init__(self, model_name: str = "m1013", update_dt_ms: int = 1):
        """
        로봇 객체를 초기화하고, 모델명에 맞는 C++ 인스턴스를 생성합니다.
        
        Args:
            model_name: 로봇 모델 이름 ("m1013", "hcr14" 등)
            update_dt_ms: 제어 루프 주기 [ms] (기본값: 1ms)
        """
        self.model_name = model_name.lower()
        
        # 🌟 C++ 팩토리 매커니즘을 통해 완벽하게 동적 할당을 받습니다.
        # 이 시점에 한화 로봇일 경우 수정한 dladdr 경로 스캔 및 .ini 동적 갱신이 백엔드에서 수행됩니다.
        self._impl = _rtb.create_robot(model_name)

    # ==============================================================
    # 🔌 연결 및 하드웨어 제어 API
    # ==============================================================

    def open_connection(self, ip: str = "192.168.1.30", port: int = 12345) -> bool:
        """로봇 컨트롤러와 TCP/IP 연결을 시도합니다."""
        return self._impl.open_connection(ip, port)

    def connect_rt(self, ip: str = "192.168.1.30", port: int = 12347) -> bool:
        """실시간 제어를 위한 런타임 제어 모드를 활성화합니다."""
        return self._impl.connect_rt(ip, port)

    def servo_on(self) -> bool:
        """로봇의 서보를 켭니다. 궤적 엔진이 자동 초기화됩니다."""
        return self._impl.servo_on()

    def servo_off(self) -> bool:
        """실시간 제어 루프를 중단하고 서보를 끕니다."""
        return self._impl.servo_off()

    def set_digital_output(self, index: int, value: bool):
        """컨트롤 박스의 디지털 출력을 제어합니다."""
        self._impl.set_digital_output(index, value)

    def get_digital_input(self, index: int) -> bool:
        """컨트롤 박스의 디지털 입력(DI) 상태를 실시간으로 읽어옵니다."""
        return self._impl.get_digital_input(index)

    # ==============================================================
    # 🔧 도구(Tool) 파라미터 및 안전 설정 API
    # ==============================================================
    
    def add_tool(self, name: str, weight: float, cog: List[float], inertia: List[float]) -> bool:
        """새로운 툴(도구)의 스펙을 컨트롤러에 등록합니다."""
        return self._impl.add_tool(name, weight, cog, inertia)

    def set_tool(self, name: str) -> bool:
        """미리 등록된 툴을 활성화하여 동역학 계산(Dynamics)에 즉시 반영합니다."""
        return self._impl.set_tool(name)

    def del_tool(self, name: str) -> bool:
        """등록된 툴을 제어기 목록에서 삭제합니다."""
        return self._impl.del_tool(name)

    def change_collision_sensitivity(self, sensitivity: float) -> bool:
        """충돌 감지 민감도를 설정합니다."""
        return self._impl.change_collision_sensitivity(sensitivity)

    # ==============================================================
    # 🚨 모니터링 및 에러 처리 API
    # ==============================================================
    
    def pop_alarm(self) -> Optional[_rtb.RobotAlarm]:
        """큐에 쌓여있는 로봇 알람(Error)을 하나 꺼내옵니다. (없으면 None)"""
        return self._impl.pop_alarm()

    # ==============================================================
    # 🚀 궤적 제어 API (Movement Commands)
    # ==============================================================

    def set_tcp(self, x: float, y: float, z: float, r: float, p: float, yaw: float):
        """툴 오프셋(TCP)을 설정합니다. (단위: m, deg)"""
        self._impl.set_tcp(x, y, z, r, p, yaw)

    def trapj(self, goal_q: npt.NDArray) -> bool:
        """관절 공간 사다리꼴 궤적 이동 명령 (Non-blocking)"""
        return self._impl.trapj(goal_q)

    def attrl(self, goal_tmat: npt.NDArray, kp: float = 50.0, target_speed: float = 0.20) -> bool:
        """작업 공간 어트랙터 기반 이동 명령
        
        Args:
            goal_tmat (NDArray): 목표 4x4 변환 행렬 (Isometry3d)
            kp (float, optional): 궤적 추종 P 게인. 기본값은 50.0.
            target_speed (float, optional): 목표 이동 속도 (m/s). 기본값은 0.20.
            
        Returns:
            bool: 명령 수락 여부 (물리적 한계 초과 시 False 반환)
        """
        return self._impl.attrl(goal_tmat, kp, target_speed)

    def align_to_floor(self, yaw_deg: float = 0.0, kp: float = 100.0) -> bool:
        """현재 위치를 유지하며 TCP를 바닥 방향(-Z)으로 정렬합니다."""
        return self._impl.align_to_floor(yaw_deg, kp)

    def align_to_front(self, kp: float = 100.0) -> bool:
        """현재 위치를 유지하며 TCP를 정면 방향으로 정렬합니다."""
        return self._impl.align_to_front(kp)

    def stop(self):
        """로봇을 즉시 부드럽게 정지시킵니다."""
        self._impl.stop()

    def solve_forward(self, q: npt.NDArray) -> npt.NDArray[np.float64]:
        """입력한 관절 각도(q)에 대한 순기구학 행렬을 계산하여 반환합니다."""
        return self._impl.solve_forward(q)

    def goal_reached(self, q_th: float = 2.0, p_th: float = 0.002, r_th: float = 3.0, v_th: float = 1.0) -> bool:
        """로봇이 목표 지점에 물리적으로 도달하고 완전히 안착(Settled)했는지 확인합니다."""
        return self._impl.goal_reached(q_th, p_th, r_th, v_th)

    # ==============================================================
    # 📊 상태 모니터링 API (Properties)
    # ==============================================================

    @property
    def angles(self) -> npt.NDArray[np.float64]:
        """실시간 로봇 관절 각도 [deg]"""
        return self._impl.angles

    @property
    def angvels(self) -> npt.NDArray[np.float64]:
        """실시간 로봇 관절 각속도 [deg/s]"""
        return self._impl.angvels

    @property
    def tmat(self) -> npt.NDArray[np.float64]:
        """실시간 TCP 포즈 행렬 (4x4)"""
        return self._impl.tmat

    @property
    def flange_tmat(self) -> npt.NDArray[np.float64]:
        """TCP 오프셋이 적용되지 않은 순수 손목(Flange) 포즈 행렬 (4x4)"""
        return self._impl.flange_tmat

    @property
    def jmat(self) -> npt.NDArray[np.float64]:
        """현재 자세의 자코비안 행렬 (6x6)"""
        return self._impl.jmat

    @property
    def task_vel(self) -> npt.NDArray[np.float64]:
        """현재 작업 공간(Task space) 속도 (6x1)"""
        return self._impl.task_vel