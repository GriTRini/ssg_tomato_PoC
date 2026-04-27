import numpy as np
import time
import random
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

# 이제 프로젝트 내 어디서든 이 방식으로 임포트 가능합니다.
from rt_py import trajectory as rc
def run_random_pid_test():
    # 1. Gazebo 통신 설정
    gz_node = Node()
    publishers = [gz_node.advertise(f"/model/m1013/joint/joint_{i}/cmd_pos", Double) for i in range(1, 7)]
    
    # 2. 로봇 모델 및 궤적 생성기 초기화
    model = rc.RobotModel("m1013")
    gen = rc.TrajGenerator()
    
    # 시작 상태 (All Zero)
    curr_q = np.zeros(6)
    gen.initialize(model, curr_q, np.zeros(6), np.zeros(6))
    
    dt = 0.001 # 1ms 제어 주기

    def send_to_gazebo(angles_deg):
        for i, angle in enumerate(angles_deg):
            msg = Double()
            msg.data = np.deg2rad(angle)
            publishers[i].publish(msg)

    print("🚀 PID 튜닝을 위한 랜덤 모션 테스트를 시작합니다. (Ctrl+C로 중단)")

    try:
        test_count = 1
        while True:
            # -90도에서 90도 사이의 랜덤 각도 생성 (6축 모두)
            target_q = np.array([random.uniform(-90.0, 90.0) for _ in range(6)])
            
            print(f"\n[Test #{test_count}] 목표 각도: {np.round(target_q, 2)}")
            gen.trapj(target_q)
            
            # 목표 도달 시까지 루프
            while not gen.goal_reached():
                loop_start = time.perf_counter()
                
                gen.update(dt)
                send_to_gazebo(gen.angles)
                
                # 1ms Busy-wait
                while (time.perf_counter() - loop_start) < dt:
                    pass
            
            print(f"✅ #{test_count} 도달 완료. 0.5초 대기...")
            time.sleep(0.5) # 도달 후 진동(Ringing)이 있는지 관찰하기 위한 대기 시간
            test_count += 1

    except KeyboardInterrupt:
        print("\n🛑 테스트 중단")

if __name__ == "__main__":
    run_random_pid_test()