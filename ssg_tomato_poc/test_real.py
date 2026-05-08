import sys
import os
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import zenoh

from rt_py.robot import create_robot

class AppleRobotController:
    def __init__(self):
        # 1. 기본 설정 및 파라미터
        self.robot = create_robot("m1013")
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
        self.cycle_count = 1  # 현재 반복 횟수 카운터
        self.is_running = True
        
        # 데이터 로그
        self.history = {"time": [], "q": [], "pos": [], "mode": [], "cycle": []}
        self.start_time = 0.0

        # ==========================================
        # 🌐 Zenoh 통신 초기화
        # ==========================================
        self.conf = zenoh.Config()
        self.z_session = zenoh.open(self.conf)
        self.z_pub_req = self.z_session.declare_publisher("detector/request")
        self.z_sub_res = self.z_session.declare_subscriber("detector/response", self.zenoh_response_callback)
        
        # Zenoh 상태 관리 변수
        self.best_target_mat = None
        self.max_score = -1.0
        self.response_received = False
        self.request_sent = False

    def zenoh_response_callback(self, sample):
        """Zenoh Subscriber 콜백: 비전 디텍터로부터 타겟 포즈를 수신합니다."""
        payload = sample.payload.to_string()
        data = payload.split(" ")
        
        if len(data) >= 18:
            try:
                score = float(data[1])
                mat_vals = list(map(float, data[2:18]))
                target_mat = np.array(mat_vals).reshape(4, 4)
                
                # 가장 스코어가 높은 타겟 저장
                if score > self.max_score:
                    self.max_score = score
                    self.best_target_mat = target_mat
                
                self.response_received = True
            except ValueError as e:
                print(f"데이터 파싱 에러: {e}")

    def log_data(self, mode_id):
        curr_t = time.time() - self.start_time
        self.history["time"].append(curr_t)
        self.history["q"].append(self.robot.angles.copy())
        self.history["pos"].append(self.robot.tmat[:3, 3].copy())
        self.history["mode"].append(mode_id)
        self.history["cycle"].append(self.cycle_count)

    def run(self):
        print("🤖 [Sequence Control] 사과 피킹 및 슬라이싱 (Zenoh 비전 연동 모드)")
        print("정지하려면 터미널에서 Ctrl+C를 누르세요.")
        
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

            # -----------------------------------------------------
            # 🔄 메인 제어 루프
            # -----------------------------------------------------
            while self.is_running:
                # 1. 목표 도달 확인 (임계값 설정: 관절 0.1도, 위치 2mm)
                if self.robot.get_goal_reached(q_th=0.1, p_th=0.002):
                    
                    # --- 시퀀스 로직 분기 ---
                    
                    # [Step 1] 홈포인트 이동 및 Zenoh 상태 초기화
                    if self.current_step == 1:
                        print(f"🔄 [사이클 {self.cycle_count}] ▶ [Step 1] 홈포인트 이동")
                        q_home = np.array([-90.0, -20.0, -70.0, 0.0, -80.0, 0.0])
                        self.robot.trapj(q_home)
                        
                        # 다음 피킹을 위해 통신 플래그 초기화
                        self.max_score = -1.0
                        self.response_received = False
                        self.request_sent = False
                        self.best_target_mat = None
                        
                        self.current_step = 2

                    # [Step 2] 비전 피킹 타겟 요청 및 하강 (Zenoh 연동)
                    elif self.current_step == 2:
                        if not self.request_sent:
                            print(f"  ▶ [Step 2] 비전 인식(Zenoh) 타겟 요청 중...")
                            flatten_mat = self.robot.flange_tmat.flatten()
                            self.z_pub_req.put(" ".join(map(str, flatten_mat)))
                            self.request_sent = True
                        
                        elif self.response_received:
                            print(f"  ✅ Pick 타겟 발견! 인식된 위치로 이동합니다.")
                            target_tmat = self.best_target_mat.copy()                            
                            self.robot.attrl(target_tmat, kp=100.0)
                            self.current_step = 3

                    # [Step 3] 홈 복귀
                    elif self.current_step == 3:
                        print(f"  ▶ [Step 3] 피킹 후 홈 복귀")
                        q_home = np.array([-90.0, -20.0, -70.0, 0.0, -80.0, 0.0])
                        self.robot.trapj(q_home)
                        self.current_step = 4

                    # [Step 4] 지그 위 대기 위치
                    elif self.current_step == 4:
                        print(f"  ▶ [Step 4] 지그 위 대기 위치 이동")
                        q_zig = np.array([-215.6, -19.69, -75.06, -6.62, -78.11, 0.0])
                        self.robot.trapj(q_zig)
                        self.current_step = 5

                    # [Step 5] 지그 안착 (통합 하강)
                    elif self.current_step == 5:
                        print(f"  ▶ [Step 5] 지그 위 사과 안착 (Z-0.1)")
                        target_tmat = self.robot.tmat.copy()
                        target_tmat[2, 3] -= 0.1 
                        self.robot.attrl(target_tmat, kp=100.0)
                        self.current_step = 6

                    # [Step 6] 90도 회전 피킹 위치 이동
                    elif self.current_step == 6:
                        print(f"  ▶ [Step 6] 90도 회전 피킹 위치 이동")
                        q_zig_90deg = np.array([-215.6, -19.69, -75.06, -6.62, -78.11, 0.0])
                        self.robot.trapj(q_zig_90deg)
                        self.current_step = 7

                    # [Step 7] 90도 회전 피킹 진행
                    elif self.current_step == 7:
                        print(f"  ▶ [Step 7] 90도 회전 피킹 진행")
                        target_tmat = self.robot.tmat.copy()
                        target_tmat[1, 3] -= 0.1
                        self.robot.attrl(target_tmat, kp=100.0)
                        self.current_step = 8

                    # [Step 8] 90도 회전 피킹 후 상승
                    elif self.current_step == 8:
                        print(f"  ▶ [Step 8] 피킹 후 상승 (Z+0.1)")
                        target_tmat = self.robot.tmat.copy()
                        target_tmat[2, 3] += 0.1
                        self.robot.attrl(target_tmat, kp=100.0)
                        self.current_step = 9

                    # [Step 9] 6번 조인트 90도 회전
                    elif self.current_step == 9:
                        print(f"  ▶ [Step 9] +90도 6번 조인트 회전")
                        target_q = self.robot.angles.copy()
                        target_q[5] += 90.0
                        self.robot.trapj(target_q)
                        self.current_step = 10

                    # [Step 10] 슬라이싱 위치 이동
                    elif self.current_step == 10:
                        print(f"  ▶ [Step 10] 슬라이싱 위치 이동")
                        q_slice_trapj = np.array([-215.6, -19.69, -75.06, -6.62, -78.11, 0.0])
                        self.robot.trapj(q_slice_trapj)
                        self.current_step = 11

                    # [Step 11] 슬라이싱 동작
                    elif self.current_step == 11:
                        print(f"  ▶ [Step 11] 슬라이싱 (Y-0.1)")
                        target_tmat = self.robot.tmat.copy()
                        target_tmat[1, 3] -= 0.1
                        self.robot.attrl(target_tmat, kp=100.0)
                        self.current_step = 12

                    # [Step 12] 반복 로직 처리
                    elif self.current_step == 12:
                        print(f"✅ [{self.cycle_count}번째 사이클] 시퀀스 동작 완료!\n")
                        self.cycle_count += 1
                        self.current_step = 1  # 1번 스텝으로 되돌려 무한 반복
                        continue

                # 실시간 데이터 로깅 (메인 루프에서 수행)
                self.log_data(self.current_step)
                time.sleep(0.002) # CPU 부하 감소를 위한 미세 대기

        except KeyboardInterrupt:
            print("\n🛑 사용자에 의해(Ctrl+C) 반복 시퀀스가 중단되었습니다.")
            self.is_running = False
        except Exception as e:
            print(f"🚨 예외 발생: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        print("\n🛑 시스템 종료 절차 시작")
        
        # Zenoh 세션 안전하게 닫기
        print("🔌 Zenoh 세션 종료...")
        self.z_session.close()
        
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
            "mode": self.history["mode"],
            "cycle": self.history["cycle"]
        })
        df.to_csv("apple_sequence_statemachine_log.csv", index=False)
        print("💾 로그 저장 완료: apple_sequence_statemachine_log.csv")

        plt.figure(figsize=(10, 5))
        plt.plot(self.history["time"], [p[2] for p in self.history["pos"]], label='Z-height (m)')
        plt.title("TCP Z-axis Motion Profile (Zenoh Linked)")
        plt.xlabel("Time (s)"); plt.ylabel("Height (m)"); plt.grid(True); plt.legend()
        plt.show()

if __name__ == "__main__":
    controller = AppleRobotController()
    controller.run()