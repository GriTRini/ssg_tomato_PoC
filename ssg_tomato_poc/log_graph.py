import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
# GUI가 없는 환경을 위해 백엔드를 'Agg'로 설정 (경고 방지)
matplotlib.use('Agg') 

# 1. 데이터 불러오기
try:
    df = pd.read_csv('apple_rt_log.csv')
except FileNotFoundError:
    print("에러: 'apple_rt_log.csv' 파일을 찾을 수 없습니다.")
    exit()

# 2. 그래프 그리기 설정 (4개의 서브플롯)
fig, axes = plt.subplots(4, 1, figsize=(12, 18), sharex=True)

# --- (1) 관절 각도 (q1 ~ q6) ---
q_cols = [f'q{i}' for i in range(1, 7)]
for col in q_cols:
    if col in df.columns:
        axes[0].plot(df['time'], df[col], label=col)
axes[0].set_ylabel('Joint Angles (deg)')
axes[0].set_title('Joint Angles Profile')
axes[0].legend(loc='upper right', ncol=3)
axes[0].grid(True, linestyle='--', alpha=0.6)

# --- (2) TCP 위치 (pos_x, pos_y, pos_z) ---
pos_cols = ['pos_x', 'pos_y', 'pos_z']
for col in pos_cols:
    if col in df.columns:
        axes[1].plot(df['time'], df[col], label=col)
axes[1].set_ylabel('Position (m)')
axes[1].set_title('TCP Position Profile')
axes[1].legend(loc='upper right')
axes[1].grid(True, linestyle='--', alpha=0.6)

# --- (3) 작동 모드 (Mode) ---
if 'mode' in df.columns:
    axes[2].step(df['time'], df['mode'], where='post', color='red', linewidth=2)
    axes[2].set_ylabel('Mode')
    axes[2].set_title('Operation Mode')
    axes[2].grid(True, linestyle='--', alpha=0.6)

# --- (4) 사이클 (Cycle) ---
if 'cycle' in df.columns:
    axes[3].plot(df['time'], df['cycle'], color='green')
    axes[3].set_ylabel('Cycle Count')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title('Execution Cycle')
    axes[3].grid(True, linestyle='--', alpha=0.6)

# 3. 레이아웃 조정 및 파일 저장
plt.tight_layout()

# 핵심 수정: plt.show() 대신 savefig 사용
save_path = 'robot_log_analysis.png'
plt.savefig(save_path) 
print(f"✅ 그래프가 성공적으로 '{save_path}'에 저장되었습니다.")