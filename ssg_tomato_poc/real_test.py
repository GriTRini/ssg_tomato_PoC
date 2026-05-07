import robot as rc
import numpy as np
import pandas as pd
import time
import matplotlib.pyplot as plt
import sys

class AppleRobotController:
    def __init__(self):
        # 1. 기본 설정 및 파라미터
        self.robot = rc.create_robot("m1013")
        self.robot_ip = "192.168.1.30"
        
        # TCP 및 툴 설정
        self.tcp_params = [0.03, 0.0, 0.16, 0.0, 0.0, 0.0]
        self.tool_info = {
            "name": "Gripper_3kg",
            "weight": 3.0,
            "cog": [0.0, 0.0, 0.0],
            "inertia": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }

        # 시퀀스 제어 변수
        self.current_step = 1
        self.is_running = True
        
        # 주요 위치 정보
        self.q_home = np.array([-90.0, -20.0, -70.0, 0.0, -80.0, 0.0])
        self.q_step5 = np.array([-215.6, -19.69, -75.06, -6.62, -78.11, 0.0])
        self.q_step10 = np.array([45.0, -30.0, -50.0, 10.0, -80.0, 0.0])
        
        # 데이터 로그
        self.history = {"time": [], "q": [], "pos": [], "mode": []}
        self.start_time = 0.0

    def log_data(self, mode_id):
        curr_t = time.time() - self.start_time
        self.history["time"].append(curr_t)
        self.history["q"].append(self.robot.angles.copy())
        self.history["pos"].append(self.robot.tmat[:3, 3].copy())
        self.history["mode"].append(mode_id)

    def run(self):
        print("🤖 [Sequence Control] 사과 피킹 및 슬라이싱 시퀀스 시작 (상태 머신 모드)")
        
        try:
            if not self.robot.open_connection(self.robot_ip):
                print(f"❌ 로봇 연결 실패 (IP: {self.robot_ip})")
                return

            self.robot.connect_rt()
            self.robot.set_tcp(*self.tcp_params)
            self.robot.add_tool(self.tool_info["name"], self.tool_info["weight"], 
                               self.tool_info["cog"], self.tool_info["inertia"])
            self.robot.set_tool(self.tool_info["name"])
            
            if not self.robot.servo_on():
                print("❌ 서보 온 실패!")
                return

            self.start_time = time.time()
            print("⚡ 로봇 준비 완료. 시퀀스 실행 중...\n")

            # 메인 제어 루프
            while self.is_running:
                # 1. 목표 도달 확인 (임계값 설정: 관절 0.1도, 위치 2mm)
                if self.robot.get_goal_reached(q_th=0.1, p_th=0.002):
                    
                    target_tmat = None
                    target_q = None
                    msg = ""
                    next_step = self.current_step

                    # --- 시퀀스 로직 분기 ---
                    
                    # [Step 1] 홈포인트 이동
                    if self.current_step == 1:
                        target_q = self.q_home
                        msg = "홈포인트 이동"
                        next_step = 2

                    # [Step 2] 피킹 위치 하강
                    elif self.current_step == 2:
                        target_tmat = self.robot.tmat.copy()
                        target_tmat[2, 3] -= 0.1
                        msg = "사과 피킹 위치 하강 (Z-0.1)"
                        next_step = 3

                    # [Step 3] 홈 복귀
                    elif self.current_step == 3:
                        target_q = self.q_home
                        msg = "피킹 후 홈 복귀"
                        next_step = 4

                    # [Step 4] 지그 방향 회전
                    elif self.current_step == 4:
                        target_q = self.q_home.copy()
                        target_q[0] = -270.0
                        msg = "지그 방향 조인트 회전"
                        next_step = 5

                    # [Step 5] 지그 위 대기 위치
                    elif self.current_step == 5:
                        target_q = self.q_step5.copy()
                        target_q[1] -= 10.0
                        target_q[2] += 15.0
                        msg = "지그 위 대기 위치 이동"
                        next_step = 6

                    # [Step 6 & 7] 지그 안착 (통합 하강)
                    elif self.current_step == 6:
                        target_tmat = self.robot.tmat.copy()
                        target_tmat[2, 3] -= 0.1 
                        msg = "지그 위 사과 안착 (Z-0.1)"
                        next_step = 8

                    # [Step 8] 90도 회전 피킹 위치 이동 및 상승
                    elif self.current_step == 8:
                        target_q = self.q_step5
                        msg = "90도 회전 피킹 지점 이동"
                        next_step = 8.1

                    elif self.current_step == 8.1:
                        target_tmat = self.robot.tmat.copy()
                        target_tmat[2, 3] += 0.1
                        msg = "피킹 후 상승 (Z+0.1)"
                        next_step = 9

                    # [Step 9] 6번 조인트 90도 회전
                    elif self.current_step == 9:
                        target_q = self.robot.angles.copy()
                        target_q[5] += 90.0
                        msg = "+90도 6번 조인트 회전"
                        next_step = 10

                    # [Step 10] 슬라이싱 위치 이동
                    elif self.current_step == 10:
                        target_q = self.q_step10
                        msg = "슬라이싱 위치 이동"
                        next_step = 11

                    # [Step 11] 슬라이싱 동작
                    elif self.current_step == 11:
                        target_tmat = self.robot.tmat.copy()
                        target_tmat[1, 3] += 0.1
                        msg = "슬라이싱 (Y+0.1)"
                        next_step = 12

                    # [Step 12] 종료
                    elif self.current_step == 12:
                        print("✅ 모든 시퀀스 동작 완료!")
                        self.is_running = False
                        continue

                    # --- 명령 전송 ---
                    if target_q is not None:
                        self.robot.trapj(target_q)
                        print(f"  ▶ [Step {self.current_step}] {msg}")
                    elif target_tmat is not None:
                        self.robot.attrl(target_tmat, kp=100.0)
                        print(f"  ▶ [Step {self.current_step}] {msg}")
                    
                    self.current_step = next_step

                # 실시간 데이터 로깅 (메인 루프에서 수행)
                self.log_data(self.current_step)
                time.sleep(0.002) # CPU 부하 감소를 위한 미세 대기

        except Exception as e:
            print(f"🚨 예외 발생: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        print("\n🛑 시스템 종료 절차 시작")
        self.robot.servo_off()
        self.robot.close_connection()
        if self.history["time"]:
            self.save_and_plot()

    def save_and_plot(self):
        df = pd.DataFrame({
            "time": self.history["time"],
            "x": [p[0] for p in self.history["pos"]],
            "y": [p[1] for p in self.history["pos"]],
            "z": [p[2] for p in self.history["pos"]],
            "mode": self.history["mode"]
        })
        df.to_csv("apple_sequence_statemachine_log.csv", index=False)
        print("💾 로그 저장 완료: apple_sequence_statemachine_log.csv")

        plt.figure(figsize=(10, 5))
        plt.plot(self.history["time"], [p[2] for p in self.history["pos"]], label='Z-height (m)')
        plt.title("TCP Z-axis Motion Profile (State Machine)")
        plt.xlabel("Time (s)"); plt.ylabel("Height (m)"); plt.grid(True); plt.legend()
        plt.show()

if __name__ == "__main__":
    controller = AppleRobotController()
    controller.run()
