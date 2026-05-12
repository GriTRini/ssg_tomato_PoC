import sys
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import zenoh
from itertools import count
from rt_py.robot import create_robot

class AppleRobotController:
    def __init__(self):
        # --- 1. 기본 설정 및 로봇 파라미터 ---
        self.robot = create_robot("m1013")
        self.robot_ip = "192.168.1.30"
        self.CONTROL_DT = 0.001  # 1ms 제어 주기
        
        self.tcp_params = [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]
        self.tool_info = {
            "name": "Gripper_3kg",
            "weight": 0.62,
            "cog": [7.62, -93.51, 167.3],
            "inertia": [0.0] * 6
        }

        # --- 2. 상태 제어 및 데이터 로깅 변수 ---
        self.current_step = 1
        self.cycle_count = 1
        self.wait_count = 0  # 특정 동작 대기용 카운터
        self.history = {"time": [], "q": [], "pos": [], "mode": [], "cycle": []}
        self.step_interval = 2
        
        # --- 3. Zenoh 통신 설정 ---
        try:
            self.conf = zenoh.Config()
            self.z_session = zenoh.open(self.conf)
            self.z_pub_req = self.z_session.declare_publisher("detector/request")
            self.z_sub_res = self.z_session.declare_subscriber("detector/response", self.zenoh_callback)
            print("✅ Zenoh 연결 성공")
        except Exception as e:
            print(f"❌ Zenoh 연결 실패: {e}")
            sys.exit(1)

        self.best_target_mat = None
        self.max_score = -1.0
        self.response_received = False
        self.request_sent = False
        self.q_home = np.array([-86.96, -31.27, -59.55, -0.18, -89.7, 0.0])

    def zenoh_callback(self, sample):
        payload = sample.payload.to_string()
        data = payload.split(" ")
        if len(data) >= 18:
            try:
                score = float(data[1])
                mat_vals = list(map(float, data[2:18]))
                target_mat = np.array(mat_vals).reshape(4, 4)
                if score > self.max_score:
                    self.max_score = score
                    self.best_target_mat = target_mat
                self.response_received = True
            except ValueError: pass

    def log_data(self, start_time):
        curr_t = time.perf_counter() - start_time
        self.history["time"].append(curr_t)
        self.history["q"].append(self.robot.angles.copy())
        self.history["pos"].append(self.robot.tmat[:3, 3].copy())
        self.history["mode"].append(self.current_step)
        self.history["cycle"].append(self.cycle_count)

    def run(self):
        if not self.robot.open_connection(self.robot_ip): return
        self.robot.connect_rt()
        self.robot.set_tcp(*self.tcp_params)
        self.robot.add_tool(self.tool_info["name"], self.tool_info["weight"], 
                           self.tool_info["cog"], self.tool_info["inertia"])
        self.robot.set_tool(self.tool_info["name"])
        self.robot.servo_on()

        print("⚡ 로봇 준비 완료. 정밀 제어 루프(1ms) 시작.")
        self.robot.trapj(self.q_home)
        
        start_time = time.perf_counter()
        next_loop_time = time.perf_counter()

        try:
            for i in count():
                # [Busy-Wait] 1ms 주기를 칼같이 맞춤
                while time.perf_counter() < next_loop_time:
                    pass
                
                # [1] 대기 카운트 처리
                if self.wait_count > 0:
                    self.wait_count -= 1
                
                
                # [2] 상태 머신 로직
                else:
                    reached = self.robot.goal_reached(q_th=0.1, p_th=0.002)
                    
                    if reached:
                        # --- Sequence Logic ---
                        if self.current_step == 1:
                            print(f"\n🔄 [Cycle {self.cycle_count}] Step 1: Home")
                            self.robot.trapj(self.q_home)
                            self.max_score, self.response_received, self.request_sent = -1.0, False, False
                            self.current_step = 2

                        elif self.current_step == 2:
                            if not self.response_received:
                                print("📡 비전 타겟 요청 중...")
                                self.z_pub_req.put(" ".join(map(str, self.robot.flange_tmat.flatten())))
                                self.robot.set_digital_output(8, False)
                                # self.response_received = True

                            elif self.response_received:
                                print("✅ 타겟 확인. 타겟 기울기에 맞춘 Approach 이동")
                                
                                # 1. 타겟의 로컬 좌표계 기준 변위 행렬 생성 (Z축으로 -0.05m 이동)
                                # (일반적으로 접근은 타겟의 Z축 반대 방향이므로 -0.05를 사용하거나, 
                                # 인식기 설정에 따라 +0.05를 사용하세요.)
                                offset_val = 0.05 
                                offset_mat = np.eye(4)
                                offset_mat[2, 3] = -offset_val  # 타겟의 로컬 Z축 방향으로 5cm 띄움
                                
                                # 2. 행렬 곱을 통해 베이스 좌표계 기준의 새로운 위치 계산
                                # [Base_T_Target] * [Target_T_Approach] = [Base_T_Approach]
                                app_pick = self.best_target_mat @ offset_mat
                                
                                # self.robot.attrl(app_pick, kp=200.0)
                                # test_pick = self.robot.tmat.copy()
                                # test_pick[2, 3] -= 0.05
                                # self.robot.attrl(test_pick, kp=200.0)
                                # pick_target = self.best_target_mat.copy()
                                # approach_pick = pick_target.copy()
                                # approach_pick = self.robot.tmat.copy()
                                # approach_pick[2, 3] -= 0.05
                                self.robot.attrl(app_pick, kp=500.0)
                                # self.wait_count = int(self.step_interval / self.CONTROL_DT)
                                self.current_step = 2.1

                        elif self.current_step == 2.1:
                            print("⬇️ 피킹 위치 하강")
                            self.robot.attrl(self.best_target_mat, kp=500.0)
                            # self.wait_count = int(self.step_interval / self.CONTROL_DT)
                            self.robot.set_digital_output(8, True)
                            self.current_step = 3

                        elif self.current_step == 3:
                            print("들기")
                            self.robot.trapj(self.q_home)
                            self.wait_count = int(self.step_interval / self.CONTROL_DT)
                            self.current_step = 4

                        elif self.current_step == 4:
                            print("지그 위치 이동")
                            q_zig_1 = np.array([-259.35, -26.67, -67.6, 4.74, -85.36, 0.0])
                            self.robot.trapj(q_zig_1)
                            self.wait_count = int(2 / self.CONTROL_DT)
                            self.current_step = 4.1

                        elif self.current_step == 4.1:
                            print("지그로 하강")
                            target_tmat = self.robot.tmat.copy()
                            target_tmat[2, 3] -= 0.07
                            self.robot.attrl(target_tmat, kp=500.0)
                            self.current_step = 4.2

                        elif self.current_step == 4.2:
                            print("사과 떨어뜨리기")
                            self.robot.set_digital_output(8, False)
                            self.current_step = 5

                        elif self.current_step == 5:
                            print("놓고 상승 및 후퇴")
                            target_tmat = self.robot.tmat.copy()
                            target_tmat[2, 3] += 0.1
                            self.robot.attrl(target_tmat, kp=500.0)
                            self.current_step = 5.11

                        elif self.current_step == 5.11:
                            print("놓고 상승 및 후퇴")
                            target_tmat = self.robot.tmat.copy()
                            target_tmat[1, 3] += 0.4
                            self.robot.attrl(target_tmat, kp=500.0)
                            self.current_step = 5.1

                        elif self.current_step == 5.1:
                            print("90도 접근")
                            q_zig_90deg = np.array([-246.74, -21.1, -139.97, 21.15, 74.06, -90.0])
                            self.robot.trapj(q_zig_90deg)
                            self.wait_count = int(1 / self.CONTROL_DT)
                            self.current_step = 5.2

                        elif self.current_step == 5.2:
                            print("90도 회전 잡으로 가기")
                            target_tmat = self.robot.tmat.copy()
                            target_tmat[1, 3] -= 0.04
                            self.robot.attrl(target_tmat, kp=500.0)
                            self.current_step = 5.3

                        elif self.current_step == 5.3:
                            print("90도 회전 후 잡기")
                            self.robot.set_digital_output(8, True)
                            self.current_step = 5.4

                        elif self.current_step == 5.4:
                            print("잡은 후 상승")
                            target_tmat = self.robot.tmat.copy()
                            target_tmat[2, 3] += 0.1
                            self.robot.attrl(target_tmat, kp=500.0)
                            self.current_step = 5.5

                        elif self.current_step == 5.5:
                            target_tmat = self.robot.tmat.copy()
                            target_tmat[0, 3] -= 0.3
                            target_tmat[2, 3] -= 0.1
                            self.robot.attrl(target_tmat, kp=500.0)
                            self.current_step = 5.6

                        elif self.current_step == 5.6:
                            target_angles = self.robot.angles.copy()
                            target_angles[5] += 90
                            self.robot.trapj(target_angles)
                            self.current_step = 5.7

                        elif self.current_step == 5.7:
                            target_tmat = self.robot.tmat.copy()
                            target_tmat[1, 3] -= 0.2
                            self.robot.attrl(target_tmat, kp=500.0)
                            self.current_step = 5.8

                        elif self.current_step == 5.8:
                            target_tmat = self.robot.tmat.copy()
                            target_tmat[2, 3] += 0.4
                            self.robot.attrl(target_tmat, kp=500.0)
                            self.current_step = 1

                # [3] 로깅 및 다음 루프 시간 계산
                self.log_data(start_time)
                next_loop_time += self.CONTROL_DT

        except KeyboardInterrupt:
            print("\n🛑 중단됨")
        finally:
            self.shutdown()

    def shutdown(self):
        self.z_session.close()
        self.robot.servo_off()
        self.robot.close_connection()
        if self.history["time"]:
            self.save_and_plot()

    def save_and_plot(self):
        # 1. 기본 history 데이터로 DataFrame 생성
        df = pd.DataFrame(self.history)

        # 2. 'q' 컬럼(리스트 형태)을 개별 컬럼으로 분리 (q1 ~ q6)
        # q_list의 각 요소가 [a, b, c, d, e, f] 형태임을 이용
        q_cols = [f'q{i+1}' for i in range(len(self.history["q"][0]))]
        df_q = pd.DataFrame(self.history["q"].tolist() if isinstance(self.history["q"], np.ndarray) else self.history["q"], 
                            columns=q_cols)

        # 3. 'pos' 컬럼도 x, y, z로 분리
        pos_cols = ['pos_x', 'pos_y', 'pos_z']
        df_pos = pd.DataFrame(self.history["pos"].tolist() if isinstance(self.history["pos"], np.ndarray) else self.history["pos"], 
                              columns=pos_cols)

        # 4. 기존 df에서 리스트 형태인 'q'와 'pos'는 제거하고 분리된 컬럼들과 합치기
        df_final = pd.concat([df.drop(columns=['q', 'pos']), df_q, df_pos], axis=1)

        # 5. CSV 저장
        file_name = "apple_rt_log.csv"
        df_final.to_csv(file_name, index=False)
        print(f"✅ 데이터가 '{file_name}'에 저장되었습니다. (컬럼: {list(df_final.columns)})")

        # 6. 그래프 출력 (Z축 프로파일)
        plt.figure(figsize=(10, 5))
        plt.plot(df_final["time"], df_final["pos_z"], label='Z-axis')
        plt.title("RT Z-Axis Profile")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.grid(True)
        plt.legend()
        plt.show()

if __name__ == "__main__":
    controller = AppleRobotController()
    controller.run()
