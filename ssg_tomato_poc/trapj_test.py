import numpy as np
import time
from itertools import count
from rt_py.robot import create_robot

# 제어 주기 설정 (1ms = 1000Hz)
CONTROL_DT = 0.001 

def run_robot_simple_cycle():
    print("🤖 [Simple Cycle Mode] Doosan m1013")
    print("💡 홈포인트와 목표 각도를 무한 반복합니다. (종료: Ctrl+C)")
    
    # 1. 로봇 설정 및 초기화
    robot = create_robot("m1013")
    robot_ip = "192.168.1.30" 
    
    # 반복할 두 지점 정의
    home_q = np.array([-86.96, -31.27, -59.55, -0.18, -89.7, 0.0])
    target_q = np.array([-258.6, -29.8, -75.02, 4.63, -72.17, 0.0])
    
    targets = [home_q, target_q]
    current_idx = 0  # 0이면 홈, 1이면 목표 지점
    
    try:
        # 로봇 연결 및 서보 온
        if not robot.open_connection(robot_ip): 
            return
        robot.connect_rt()
        robot.servo_on()
        
        # 처음 동작 시작 (홈으로 이동)
        print(f"🚀 이동 시작: {'Home' if current_idx == 0 else 'Target'}")
        robot.trapj(targets[current_idx])
        
        next_loop_time = time.perf_counter()
        wait_count = 0

        # -----------------------------------------------------
        # 🔄 [Main Loop] (1ms Tick)
        # -----------------------------------------------------
        for i in count():
            
            # [1] 도착 후 대기 시간 처리 (wait_count)
            if wait_count > 0:
                wait_count -= 1
            
            # [2] 목표 도달 확인 및 다음 지점으로 전환
            else:
                # 목표 각도에 도달했는지 확인 (임계값 0.1도)
                if robot.goal_reached(q_th=0.1):
                    print(f"✅ 도달 완료: {'Home' if current_idx == 0 else 'Target'}")
                    
                    # 인덱스 전환 (0 -> 1, 1 -> 0)
                    current_idx = 1 - current_idx
                    
                    # 다음 목표로 trapj 실행
                    print(f"🔄 다음 목표로 이동: {'Home' if current_idx == 0 else 'Target'}")
                    robot.trapj(targets[current_idx])
                    
                    # 도달 후 너무 바로 움직이지 않게 약간의 휴지 시간 부여 (예: 0.5초 = 500 ticks)
                    wait_count = 0

            # 🌟 [Real-time Busy-Wait] 1ms 주기 동기화
            next_loop_time += CONTROL_DT
            while time.perf_counter() < next_loop_time:
                pass 

    except KeyboardInterrupt:
        print("\n🛑 [STOP] 사용자가 중지하였습니다.")
    except Exception as e:
        print(f"\n🚨 에외 발생: {e}")
    finally:
        print("\n🔌 안전 종료 중...")
        try:
            robot.stop()
            time.sleep(0.1)
            robot.servo_off()
        except: pass
        robot.close_connection()
        print("👋 종료되었습니다.")

if __name__ == "__main__":
    run_robot_simple_cycle()