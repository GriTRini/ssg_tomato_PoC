import rt_control_cpp_impl as rc
import numpy as np
import pandas as pd
import time
import matplotlib.pyplot as plt
import signal

def run_real_robot():
    print("🤖 [Interactive Control] Doosan m1013 Pick & Place 테스트 시작")
    
    # 1. 로봇 및 설정 초기화
    robot = rc.create_robot("m1013")
    robot_ip = "192.168.1.30" 
    
    # TCP 오프셋 (C++ 코드 기준: Z 381.9mm 및 X 오프셋 반영)
    # 실제 장착된 그리퍼 길이에 맞춰 조정하세요.
    tcp_x, tcp_y, tcp_z = -0.029, 0.0, 0.3819
    target_ori = [180.0, 0.0, 90.0] # R, P, Yaw (특이점 회피를 위해 Yaw 90도)

    history = {"time": [], "q": [], "pos": [], "mode": []}
    start_time = [0.0] # 리스트로 선언하여 내부 함수에서 접근 가능하게 설정

    def log_data(mode_id):
        curr_t = time.time() - start_time[0]
        history["time"].append(curr_t)
        history["q"].append(robot.angles.copy())
        history["pos"].append(robot.tmat[:3, 3].copy())
        history["mode"].append(mode_id)

    try:
        # 2. 연결 및 서보 온
        if not robot.open_connection(robot_ip):
            print(f"❌ 로봇 연결 실패 (IP: {robot_ip})")
            return

        robot.connect_rt()
        robot.set_tcp(tcp_x, tcp_y, tcp_z, 0.0, 0.0, 0.0)
        
        if not robot.servo_on():
            print("❌ 서보 온 실패!")
            return

        start_time[0] = time.time()
        print("⚡ 로봇 준비 완료.")

        # ---------------------------------------------------------
        # [Step 1] 초기 자세 (Home) 이동
        # ---------------------------------------------------------
        print("\n[Step 1] 초기 자세로 이동 중 (J1, J3, J5 -90도)...")
        target_q = np.array([-90.0, 0.0, -90.0, 0.0, -90.0, 0.0])
        robot.trapj(target_q)
        while not robot.goal_reached(q_th=0.1):
            log_data(1)
            time.sleep(0.01)

        # [Step 2] 바닥 정렬
        print("[Step 2] 툴 팁 바닥 정렬 (Yaw: 90.0)...")
        robot.align_to_floor(yaw_deg=90.0, kp=50.0)
        while not robot.goal_reached(r_th=0.5):
            log_data(2)
            time.sleep(0.01)

        # ---------------------------------------------------------
        # [Step 3] 인터랙티브 Pick & Place 루프
        # ---------------------------------------------------------
        print("\n" + "="*50)
        print("🚀 좌표를 입력하세요. (단위: m, 예: 0.5 -0.1 0.2)")
        print("종료하려면 'q'를 입력하세요.")
        print("="*50)

        while True:
            # --- PICK PHASE ---
            val = input("\n🟩 [PICK] 좌표 X Y Z: ")
            if val.lower() == 'q': break
            try:
                px, py, pz = map(float, val.split())
            except: continue

            # 시퀀스 실행 (Approach -> Reach -> Grip -> Retract)
            for sub_step, z_off, grip in [("App", 0.05, None), ("Reach", 0.0, True), ("Retract", 0.1, None)]:
                print(f"  ▶ Pick {sub_step} 이동 중...")
                # AttrL 제어를 위해 목표 행렬 생성 (회전은 고정)
                target_mat = robot.tmat.copy()
                target_mat[:3, 3] = [px, py, pz + z_off]
                # 주의: 실제 SDK의 attrl 함수가 T-Matrix를 받는지 확인 필요
                robot.attrl(target_mat, kp=40.0) 
                
                while not robot.goal_reached(p_th=0.002):
                    log_data(3)
                    time.sleep(0.01)
                
                if grip is True:
                    print("  🧲 그리퍼 ON (IO 9)"); robot.set_digital_output(9, True); time.sleep(0.5)

            # --- PLACE PHASE ---
            val = input("🟦 [PLACE] 좌표 X Y Z: ")
            if val.lower() == 'q': break
            try:
                lx, ly, lz = map(float, val.split())
            except: continue

            for sub_step, z_off, grip in [("App", 0.05, None), ("Reach", 0.0, False), ("Retract", 0.1, None)]:
                print(f"  ▶ Place {sub_step} 이동 중...")
                target_mat = robot.tmat.copy()
                target_mat[:3, 3] = [lx, ly, lz + z_off]
                robot.attrl(target_mat, kp=40.0)

                while not robot.goal_reached(p_th=0.002):
                    log_data(4)
                    time.sleep(0.01)

                if grip is False:
                    print("  👐 그리퍼 OFF (IO 9)"); robot.set_digital_output(9, False); time.sleep(0.5)

            print("✅ 1회 사이클 완료!")

    except Exception as e:
        print(f"🚨 예외 발생: {e}")

    finally:
        print("\n🛑 시스템 종료 절차 시작")
        robot.set_digital_output(9, False)
        robot.servo_off()
        robot.close_connection()

    # 데이터 저장 및 시각화 (기존 코드 유지)
    if history["time"]:
        save_and_plot(history)

def save_and_plot(history):
    df = pd.DataFrame({
        "time": history["time"],
        "x": [p[0] for p in history["pos"]],
        "y": [p[1] for p in history["pos"]],
        "z": [p[2] for p in history["pos"]],
        "mode": history["mode"]
    })
    df.to_csv("real_robot_interactive_log.csv", index=False)
    print("💾 로그 저장 완료: real_robot_interactive_log.csv")

    plt.figure(figsize=(10, 5))
    plt.plot(history["time"], [p[2] for p in history["pos"]], label='Z-height (m)')
    plt.title("TCP Z-axis Motion Profile")
    plt.xlabel("Time (s)"); plt.ylabel("Height (m)"); plt.grid(True); plt.legend()
    plt.show()

if __name__ == "__main__":
    run_real_robot()