import rt_control_cpp_impl as rc
import numpy as np
import pandas as pd
import time
import matplotlib.pyplot as plt

def run_real_robot():
    print("🤖 [Sequence Control] 사과 피킹 및 슬라이싱 시퀀스 테스트 시작")
    
    # 1. 로봇 객체 생성
    robot = rc.create_robot("m1013")
    robot_ip = "192.168.1.30" 
    
    # --- 설정 파라미터 ---
    # TCP 오프셋 (단위: m)
    tcp_x, tcp_y, tcp_z = 0.03, 0.0, 0.16
    
    # 툴(Tool) 정보 (무게 3kg, 무게중심 0.0, 관성모멘트 0.0)
    tool_name = "Gripper_3kg"
    tool_weight = 3.0
    tool_cog = [0.0, 0.0, 0.0]
    tool_inertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # 카운트 기반 대기 설정
    MAX_COUNT = 3000000  # 무한 루프 방지용 최대 카운트
    LOG_INTERVAL = 1000  # sleep이 없으므로 로그 저장 빈도 조절 (메모리 폭발 방지)
    # -------------------

    history = {"time": [], "q": [], "pos": [], "mode": []}
    start_time = [0.0]

    def log_data(mode_id):
        curr_t = time.time() - start_time[0]
        history["time"].append(curr_t)
        history["q"].append(robot.angles.copy())
        history["pos"].append(robot.tmat[:3, 3].copy())
        history["mode"].append(mode_id)

    try:
        # 2. 연결 및 하드웨어 제어
        if not robot.open_connection(robot_ip):
            print(f"❌ 로봇 연결 실패 (IP: {robot_ip})")
            return

        robot.connect_rt()
        
        # [툴 및 TCP 셋팅 진행]
        robot.set_tcp(tcp_x, tcp_y, tcp_z, 0.0, 0.0, 0.0)
        print(f"✅ TCP 셋팅 완료 (X: {tcp_x}m, Z: {tcp_z}m)")

        robot.add_tool(tool_name, tool_weight, tool_cog, tool_inertia)
        robot.set_tool(tool_name)
        print(f"✅ Tool 셋팅 완료 (이름: {tool_name}, 무게: {tool_weight}kg)")
        
        if not robot.servo_on():
            print("❌ 서보 온 실패!")
            return

        start_time[0] = time.time()
        print("⚡ 로봇 준비 완료. 시퀀스를 시작합니다.\n")

        # ---------------------------------------------------------
        # [Step 1] 지정된 홈 포인트 이동 (관절 제어)
        # ---------------------------------------------------------
        q_home = np.array([-90.0, -20.0, -70.0, 0.0, -80.0, 0.0])
        print("  ▶ [Mode 1] 홈포인트로 이동 중...")
        robot.trapj(q_home)
        count = 0
        while not robot.goal_reached(q_th=0.1) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(1)
            count += 1

        # ---------------------------------------------------------
        # [Step 2] 사과 피킹 위치로 하강 (포지션 제어)
        # ---------------------------------------------------------
        target_mat = robot.tmat.copy()
        target_mat[2, 3] -= 0.05  # 피킹을 위해 Z축 하강
        print("  ▶ [Mode 2] 사과 피킹 위치로 이동 (Z축 하강) 중...")
        robot.attrl(target_mat, kp=40.0)
        count = 0
        while not robot.goal_reached(p_th=0.002) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(2)
            count += 1

        # ---------------------------------------------------------
        # [Step 3] 다시 홈포인트로 복귀 (관절 제어)
        # ---------------------------------------------------------
        print("  ▶ [Mode 3] 피킹 후 홈포인트로 복귀 중...")
        robot.trapj(q_home)
        count = 0
        while not robot.goal_reached(q_th=0.1) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(3)
            count += 1

        # ---------------------------------------------------------
        # [Step 4] 1번 조인트를 회전하여 지그 방향으로 이동 (관절 제어)
        # ---------------------------------------------------------
        q_step4 = q_home.copy()
        q_step4[0] = 0.0
        print("  ▶ [Mode 4] 지그 방향으로 1번 조인트 회전 중...")
        robot.trapj(q_step4)
        count = 0
        while not robot.goal_reached(q_th=0.1) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(4)
            count += 1

        # ---------------------------------------------------------
        # [Step 5] 지그 바로 위 위치로 이동 (관절 제어)
        # ---------------------------------------------------------
        q_step5 = q_step4.copy()
        q_step5[1] -= 10.0  # 지그 위로 접근하기 위한 2,3번 조인트 각도 조정 예시
        q_step5[2] += 15.0
        print("  ▶ [Mode 5] 지그 바로 위 대기 위치로 이동 중...")
        robot.trapj(q_step5)
        count = 0
        while not robot.goal_reached(q_th=0.1) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(5)
            count += 1

        # ---------------------------------------------------------
        # [Step 6] 지그 위에 사과 안착 (포지션 제어)
        # ---------------------------------------------------------
        target_mat = robot.tmat.copy()
        target_mat[2, 3] -= 0.05  # 살짝 내려놓기
        print("  ▶ [Mode 6] 지그 위에 사과 안착 중...")
        robot.attrl(target_mat, kp=40.0)
        count = 0
        while not robot.goal_reached(p_th=0.002) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(6)
            count += 1

        # ---------------------------------------------------------
        # [Step 7] 90도 회전된 위치 피킹을 위한 이동 (관절 제어)
        # ---------------------------------------------------------
        q_step7 = np.array([30.0, -10.0, -60.0, 0.0, -90.0, 90.0]) # 6번 관절 90도 포함된 특정 포즈
        print("  ▶ [Mode 7] 90도 회전된 다른 위치의 피킹 지점으로 이동 중...")
        robot.trapj(q_step7)
        count = 0
        while not robot.goal_reached(q_th=0.1) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(7)
            count += 1

        # ---------------------------------------------------------
        # [Step 8] 해당 위치에서 10cm 상승 (포지션 제어)
        # ---------------------------------------------------------
        target_mat = robot.tmat.copy()
        target_mat[2, 3] += 0.10  # 10cm = 0.1m
        print("  ▶ [Mode 8] 피킹 후 10cm(0.1m) 상승 중...")
        robot.attrl(target_mat, kp=40.0)
        count = 0
        while not robot.goal_reached(p_th=0.002) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(8)
            count += 1

        # ---------------------------------------------------------
        # [Step 9] 6번 조인트만 90도 회전 (관절 제어)
        # ---------------------------------------------------------
        q_step9 = robot.angles.copy()
        q_step9[5] += 90.0
        print("  ▶ [Mode 9] 6번 조인트 단독 +90도 회전 중...")
        robot.trapj(q_step9)
        count = 0
        while not robot.goal_reached(q_th=0.1) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(9)
            count += 1

        # ---------------------------------------------------------
        # [Step 10] 슬라이싱 위치로 이동 (관절 제어)
        # ---------------------------------------------------------
        q_step10 = np.array([45.0, -30.0, -50.0, 10.0, -80.0, 0.0])
        print("  ▶ [Mode 10] 사과 슬라이싱 위치로 이동 중...")
        robot.trapj(q_step10)
        count = 0
        while not robot.goal_reached(q_th=0.1) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(10)
            count += 1

        # ---------------------------------------------------------
        # [Step 11] 해당 위치에서 Y축 방향으로 0.1m 이동 (포지션 제어)
        # ---------------------------------------------------------
        target_mat = robot.tmat.copy()
        target_mat[1, 3] += 0.1
        print("  ▶ [Mode 11] 슬라이싱 동작 (Y축 0.1m 직선 이동) 중...")
        robot.attrl(target_mat, kp=40.0)
        count = 0
        while not robot.goal_reached(p_th=0.002) and count < MAX_COUNT:
            if count % LOG_INTERVAL == 0: log_data(11)
            count += 1

        print("✅ 모든 시퀀스 동작 완료!")

    except Exception as e:
        print(f"🚨 예외 발생: {e}")

    finally:
        print("\n🛑 시스템 종료 절차 시작")
        robot.servo_off()
        robot.close_connection()

    # 데이터 저장 및 시각화
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
    df.to_csv("real_robot_sequence_log.csv", index=False)
    print("💾 로그 저장 완료: real_robot_sequence_log.csv")

    plt.figure(figsize=(10, 5))
    plt.plot(history["time"], [p[2] for p in history["pos"]], label='Z-height (m)')
    plt.title("TCP Z-axis Motion Profile")
    plt.xlabel("Time (s)"); plt.ylabel("Height (m)"); plt.grid(True); plt.legend()
    plt.show()

if __name__ == "__main__":
    run_real_robot()