import json
import csv
from pathlib import Path

# 현재 스크립트 파일의 위치를 기준으로 프로젝트 루트 경로 파악
current_dir = Path(__file__).resolve().parent
# 만약 scripts 폴더 안에 이 파일이 있다면 상위 폴더로 이동 (구조에 맞게 조정)
project_root = current_dir.parent 

# 절대 경로 또는 프로젝트 루트 기준으로 파일 지정
plan_path = project_root / "out" / "paper_maze_pen_plan.json"
plan = json.loads(plan_path.read_text())

# 1. 캘리브레이션 설정 파일 로드
# calibration_path = project_root / "configs" / "calibration_TEMPLATE.json"  # 실제
calibration_path = project_root / "configs" / "my_verified_calibration.json" # 테스트 좌표 (실제 로봇 사용 금지)
calibration_data = json.loads(calibration_path.read_text())

# 설정 파일에서 값 가져오기 (키 이름이 다를 경우 실제 JSON 내부 구조에 맞게 수정 필요)
origin = calibration_data["origin_base_mm"]
x_span = calibration_data["x_span_base_mm"]
y_span = calibration_data["y_span_base_mm"]

rows = []

for step, item in enumerate(plan["pen_sequence"]):
    u, v = item["xy"]

    xyz = [
        origin[i] + u * x_span[i] + v * y_span[i]
        for i in range(3)
    ]

    rows.append([
        step,
        item["action"],
        *xyz,
        item["pen"]
    ])

output = project_root / "out" / "maze_coordinates_SIMULATION_ONLY.csv"
output.parent.mkdir(exist_ok=True)

with output.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "step", "action", "x", "y", "z", "pen"
    ])
    writer.writerows(rows)

print(f"[SIMULATION ONLY] {len(rows)} coordinates generated")
print(f"First: {rows[0]}")
print(f"Last:  {rows[-1]}")
print(f"Saved: {output}")
print("[SAFETY] Do not send these coordinates to a robot")
