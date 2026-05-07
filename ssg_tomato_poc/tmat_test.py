import numpy as np
import time
import os
import sys

# 프로젝트 경로 설정 (bashrc 설정이 안 되어 있을 경우를 대비)
sys.path.append('/usr/lib/python3/dist-packages')

from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from rt_py import trajectory as rc 

def get_initial_tmat():
    # 1. Gazebo 통신 및 모델 초기화
    gz_node = Node()
    publishers = [gz_node.advertise(f"/model/m1013/joint/joint_{i}/cmd_pos", Double) for i in range(1, 7)]
    
    model = rc.RobotModel("m1013")
    gen = rc.TrajGenerator()
    
    # 0도 상태에서 시작
    start_q = np.zeros(6)
    gen.initialize(model, start_q, np.zeros(6), np.zeros(6))
    
    dt = 0.001 # 1ms

    def send_to_gazebo(angles_deg):
        for i, angle in enumerate(angles_deg):
            msg = Double()
            msg.data = np.deg2rad(angle)
            publishers[i].publish(msg)

    # --- 모션 시작 ---
    
    # Step 1: trapj를 이용하여 목표 각도로 이동
    target_q = np.array([-90.0, 0.0, -90.0, 0.0, -90.0, 0.0])
    print(f"🔄 목표 각도 {target_q}로 이동 시작...")
    
    gen.trapj(target_q)
    
    while not gen.goal_reached():
        loop_start = time.perf_counter()
        
        gen.update(dt)
        send_to_gazebo(gen.angles)
        
        while (time.perf_counter() - loop_start) < dt:
            pass
            
    print("✅ 목표 각도 도달 완료")

    # Step 2: 이동 완료 후 현재의 T-Matrix 추출
    # gen.update를 통해 내부 행렬이 최신화된 상태입니다.
    current_tmat = gen.tmat.copy()
    
    print("\n📍 [추출된 T-Matrix]")
    print(current_tmat)
    
    # 좌표값만 따로 보기 편하게 출력 (단위: m)
    pos = current_tmat[:3, 3]
    print(f"\n📌 현재 End-Effector 위치 (x, y, z): {np.round(pos, 4)}")
    
    return current_tmat

if __name__ == "__main__":
    try:
        get_initial_tmat()
    except KeyboardInterrupt:
        print("\n🛑 중단되었습니다.")