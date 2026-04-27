import numpy as np
import time
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
import trajectory as rc 

def run_motion_test():
    gz_node = Node()
    publishers = [gz_node.advertise(f"/model/m1013/joint/joint_{i}/cmd_pos", Double) for i in range(1, 7)]
    
    model = rc.RobotModel("m1013")
    gen = rc.TrajGenerator()
    
    # [중요] 실제 로봇이 0도 상태이므로 0으로 초기화
    start_q = np.zeros(6)
    gen.initialize(model, start_q, np.zeros(6), np.zeros(6))
    
    dt = 0.001 

    def send_to_gazebo(angles_deg):
        for i, angle in enumerate(angles_deg):
            msg = Double()
            msg.data = np.deg2rad(angle)
            publishers[i].publish(msg)

    def wait_motion(label):
        print(f"🔄 {label} 이동 중...")
        while not gen.goal_reached():
            loop_start = time.perf_counter()
            gen.update(dt)
            send_to_gazebo(gen.angles)
            while (time.perf_counter() - loop_start) < dt:
                pass
        print(f"✅ {label} 완료")

    # --- 테스트 시나리오 시작 ---

    # Step 0: All Zero -> (-90, 0, -90, 0, -90, 0) 로 초기 자세 잡기
    initial_pose_q = np.array([-90.0, 0.0, -90.0, 0.0, -90.0, 0.0])
    gen.trapj(initial_pose_q)
    wait_motion("Step 0: 초기 자세 설정 (-90, 0, -90, 0, -90, 0)")

    # Step 0-1: 이동 완료 후의 T-Matrix 획득 (이후 Matrix 모션의 기준점)
    gen.update(dt)
    base_tmat = gen.tmat.copy()
    print(f"📍 기준 위치(T-Matrix):\n{base_tmat}")

    # Step 1: y축 방향으로 +0.1m 이동 (base_tmat 기준 상대이동)
    target_tmat_1 = base_tmat.copy()
    target_tmat_1[1, 3] += 0.1
    gen.attrl(target_tmat_1)
    wait_motion("Step 1: Y +0.1m 이동")

    # Step 2: z축을 0.05m 위치(절대좌표)로 이동
    target_tmat_2 = target_tmat_1.copy()
    target_tmat_2[2, 3] = 0.05  
    gen.attrl(target_tmat_2)
    wait_motion("Step 2: Z 0.05m 하강")

    # Step 3: 다시 Step 1의 Y축 위치로 복귀 (Z는 원래 높이로)
    target_tmat_3 = target_tmat_1.copy() 
    gen.attrl(target_tmat_3)
    wait_motion("Step 3: Y 복귀 및 Z 상승")

    # Step 4: 1번 조인트만 0도로 변경
    target_q_4 = gen.angles.copy()
    target_q_4[0] = 0.0
    gen.trapj(target_q_4)
    wait_motion("Step 4: J1 -> 0도")

    # Step 5: 해당 위치에서 다시 z축 기준으로 0.01m 이동 (절대좌표)
    gen.update(dt)
    target_tmat_5 = gen.tmat.copy()
    target_tmat_5[2, 3] = 0.01
    gen.attrl(target_tmat_5)
    wait_motion("Step 5: 최종 Z 0.01m 이동")

    print("🏁 모든 모션 테스트 시퀀스 종료")

if __name__ == "__main__":
    run_motion_test()