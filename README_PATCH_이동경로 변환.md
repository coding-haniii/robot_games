# 미로 펜 경로 JSON → 로봇 좌표 확인용 변환기 (운동 명령 없음)

`maze_pen_executor/scripts/preview_pen_plan.py`를 기존 `robot_game_project`에 복사합니다.

```bash
cd /home/khe/ros2_ws/src/robot_game_project
python3 scripts/preview_pen_plan.py --plan out/paper_maze_pen_plan.json
```

이때는 계획의 첫/마지막 명령과 경로점 개수만 보여주고, 로봇을 움직이지 않습니다.

현장 교시/측정으로 다음을 획득하고 `configs/calibration_TEMPLATE.json`를 복사해 실제 측정값을 입력하세요:

- `origin_base_mm`: **보정된 미로 이미지 좌상단**이 실제 종이에서 위치한 TCP 기준 Base XYZ
- `x_span_base_mm`: 이미지 전체 가로 폭에 해당하는 Base XYZ 벡터
- `y_span_base_mm`: 이미지 전체 세로 높이에 해당하는 Base XYZ 벡터
- `pen_contact_z_base_mm`: 펜 끝이 종이와 닿을 때 절대 Base Z (수평 테이블 가정)
- `pen_up_z_base_mm`: 종이보다 10mm 이상 높은 안전 Base Z (수평 테이블 가정)
- `tool_orientation_deg`: 두산 컨트롤러 `posx`에서 요구하는 검증된 실제 TCP 방향 각. **단순 RPY 가정 금지**.
- `measured_and_verified`: 현장 검증 후에만 `true`

**중요:** 이 단순 평면 가정은 기울어진 종이, 카메라 광학왜곡, 3D 곡면, TCP 변위가 있는 경우 유효하지 않습니다. 이미지 좌상단은 원근 보정 후의 영역이며 카메라 모서리 클릭이 바뀌면 다시 보정해야 합니다. Z는 수평판 가정으로 고정됩니다.

```bash
python3 scripts/preview_pen_plan.py \
  --plan out/paper_maze_pen_plan.json \
  --calibration configs/my_verified_calibration.json
head -10 out/maze_robot_base_preview.csv
```

CSV는 경로 검토용으로만 사용합니다. 실제 E0509 제어를 시작하려면 ROS 2 Jazzy의 `DSR_ROBOT2` 연결 및 모션 실행기, IK/작업공간·충돌 검사, 펜 분리 접근/이탈과 힘제어, 로봇 안전정지, RH-P12-RN-A 펜 파지 및 TCP 검증, 펜 접촉이 없는 공중 경로 테스트 등을 별도로 완료해야 합니다.
