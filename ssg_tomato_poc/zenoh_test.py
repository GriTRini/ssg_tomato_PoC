import numpy as np
import time
import sys
import zenoh
from itertools import count
from rt_py.robot import create_robot

# 제어 주기 설정 (1ms = 1000Hz)
CONTROL_DT = 0.001 

def run_real_robot_detection_loop():
    print("🤖 [Dynamic Pick & Place Mode] Doosan m1013")
    print("💡 물건을 집은 후 홈으로 복귀하고, 새로운 위치를 받아 내려놓습니다.")
    
    # 1. Zenoh 세션 및 설정
    conf = zenoh.Config()
    z_session = zenoh.open(conf)
    z_pub_req = z_session.declare_publisher("detector/request")
    
    best_target_mat = None
    max_score = -1.0
    response_received = False

    def response_callback(sample):
        nonlocal best_target_mat, max_score, response_received
        
        payload = sample.payload.to_string()
        data = payload.split(" ")
        
        if len(data) >= 18:
            try:
                score = float(data[1])
                mat_vals = list(map(float, data[2:18]))
                target_mat = np.array(mat_vals).reshape(4, 4)
                
                if score > max_score:
                    max_score = score
                    best_target_mat = target_mat
                
                response_received = True
                
            except ValueError as e:
                print(f"데이터 파싱 에러: {e}")

    z_sub_res = z_session.declare_subscriber("detector/response", response_callback)

    # 2. 로봇 설정 및 초기화
    robot = create_robot("m1013")
    robot_ip = "192.168.1.30" 
    home_q = np.array([-90.0, 0.0, -90.0, 0.0, -90.0, 0.0])
    tcp_offset = [0.0, 0.029, 0.3819]
    
    try:
        if not robot.open_connection(robot_ip): 
            return
        robot.connect_rt()
        
        tool_name = "Gripper_A"
        weight = 5.19
        cog = [10.780, 8.110, -15.430]
        inertia = [0.0] * 6 
        
        print(f"\n🔧 [{tool_name}] 툴 파라미터를 등록합니다.")
        if robot.add_tool(tool_name, weight, cog, inertia):
            robot.set_tool(tool_name)
        
        robot.set_tcp(*tcp_offset, 0.0, 0.0, 0.0)
        robot.servo_on()

        # 루프 제어 변수
        current_step = 0 
        wait_count = 0
        pick_target = None
        place_target = None
        
        robot.trapj(home_q)
        next_loop_time = time.perf_counter()
        
        # -----------------------------------------------------
        # 🔄 [Main Logic Loop] (1ms Tick)
        # -----------------------------------------------------
        for i in count():
            
            # [1] 지연 카운트 처리
            if wait_count > 0:
                wait_count -= 1
            
            # [2] 카운트 완료 시 상태 머신 작동
            else:
                is_reached = robot.goal_reached(q_th=0.1, p_th=0.002)

                if is_reached:
                    # ==========================================
                    # 🟢 PHASE 1: PICK (물건 집기)
                    # ==========================================
                    if current_step == 0:
                        robot.set_digital_output(8, False) # 초기화
                        max_score, response_received, best_target_mat = -1.0, False, None
                        print("\n🏠 홈 도달. [PICK] 물건 위치 탐색을 시작합니다...")
                        current_step = 1

                    elif current_step == 1:
                        if not response_received:
                            flatten_mat = robot.flange_tmat.flatten()
                            z_pub_req.put(" ".join(map(str, flatten_mat)))
                        else:
                            print(f"✅ Pick 타겟 발견! 상단(+5cm) 위치로 진입합니다.")
                            pick_target = best_target_mat.copy() # 위치 저장
                            approach_pick = pick_target.copy()
                            approach_pick[2, 3] += 0.05
                            robot.attrl(approach_pick, kp=500.0)
                            current_step = 2

                    elif current_step == 2:
                        print("⬇️ 물건을 잡기 위해 하강합니다.")
                        robot.attrl(pick_target, kp=500.0)
                        current_step = 3

                    elif current_step == 3:
                        print("🧲 석션 ON! 진공 생성 대기 (300ms)...")
                        robot.set_digital_output(8, True)
                        wait_count = int(0.3 / CONTROL_DT) 
                        current_step = 4

                    elif current_step == 4:
                        print("⬆️ 진공 완료! 물건을 잡고 10cm 상승합니다.")
                        lift_mat = pick_target.copy()
                        lift_mat[2, 3] += 0.10
                        robot.attrl(lift_mat, kp=500.0)
                        current_step = 5

                    # ==========================================
                    # 🟡 PHASE 2: HOME 복귀 (물건 쥔 상태)
                    # ==========================================
                    elif current_step == 5:
                        print("🏠 물건을 쥔 상태로 홈 복귀 중...")
                        robot.trapj(home_q)
                        current_step = 6

                    # ==========================================
                    # 🔴 PHASE 3: PLACE (물건 놓기)
                    # ==========================================
                    elif current_step == 6:
                        max_score, response_received, best_target_mat = -1.0, False, None
                        print("\n🏠 홈 도달. [PLACE] 내려놓을 위치 탐색을 시작합니다...")
                        current_step = 7

                    elif current_step == 7:
                        if not response_received:
                            flatten_mat = robot.flange_tmat.flatten()
                            z_pub_req.put(" ".join(map(str, flatten_mat)))
                        else:
                            print(f"✅ Place 타겟 발견! 상단(+5cm) 위치로 진입합니다.")
                            place_target = best_target_mat.copy() # 위치 저장
                            approach_place = place_target.copy()
                            approach_place[2, 3] += 0.15
                            robot.attrl(approach_place, kp=500.0)
                            current_step = 8

                    elif current_step == 8:
                        print("⬇️ 물건을 놓기 위해 하강합니다.")
                        place_target = best_target_mat.copy() # 위치 저장
                        place_target_1 = place_target.copy()
                        place_target_1[2, 3] += 0.07
                        robot.attrl(place_target_1, kp=500.0)
                        current_step = 9

                    elif current_step == 9:
                        print("💨 석션 OFF! 진공 파기 대기 (300ms)...")
                        robot.set_digital_output(8, False)
                        wait_count = int(0.3 / CONTROL_DT)
                        current_step = 10

                    elif current_step == 10:
                        print("⬆️ 공기 배출 완료! 5cm 상승합니다.")
                        retract_mat = place_target.copy()
                        retract_mat[2, 3] += 0.15
                        robot.attrl(retract_mat, kp=500.0)
                        current_step = 11

                    # ==========================================
                    # 🔵 PHASE 4: HOME 복귀 (종료)
                    # ==========================================
                    elif current_step == 11:
                        print("🏠 사이클 완료. 다시 홈으로 복귀합니다.")
                        robot.trapj(home_q)
                        current_step = 0

            # 🌟 [Busy-Wait 제어]
            next_loop_time += CONTROL_DT
            while time.perf_counter() < next_loop_time:
                pass 

    except KeyboardInterrupt:
        print("\n🛑 [STOP] 사용자가 Ctrl+C를 입력하여 종료합니다.")
    except Exception as e:
        print(f"\n🚨 예외 발생: {e}")
    finally:
        print("\n🔌 안전 종료 절차 시작...")
        z_session.close()
        try:
            robot.stop()
            robot.set_digital_output(9, False) 
            time.sleep(0.1) 
            robot.servo_off()
        except: pass
        robot.close_connection()
        print("👋 프로그램이 정상 종료되었습니다.")

if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    run_real_robot_detection_loop()