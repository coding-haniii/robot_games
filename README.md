# Vision Robot Games — Ubuntu 24.04 / Doosan E0509 / ROBOTIS RH-P12-RN(A)

**Status: functional PC mock prototype and guarded hardware adapter, NOT a production-certified robot cell.**

## Architecture

- Action card (1/OX/TICTACTOE, 2/MAZE, 3/EXIT) -> OCR recognizer -> dispatcher.
- Tic-tac-toe: minimax O player; image parser for simple printed, fronto-parallel boards only; drawn O.
- Maze: arbitrary newly generated perfect mazes, or black-wall / white-path raster input with CLI start/end pixels; erosion for clearance; BFS pathfinding; visualization.
- Robot: MockRobot PC renderer/log, optional DSR_ROBOT2 guard requiring commissioning and deliberate enable.
- RH-P12-RN(A): optional independent USB-RS485 driver; not used to draw; requires **physical pen holder/tool** if pen drawing desired.
- On robot win: operator-taught straight-arm celebration pose; end: operator-taught home pose. Stop action returns home only if safe to move.

**Important:** This prototype does **not** have autonomous human turn detection, game-loop state management, camera pose estimation, marker-based homography, collision checking, safety PLC integration, or arbitrarily complex illustrated maze segmentation. The physical integration is a commissioning scaffold. Optical maze detection needs clear corridor contrast and start/end identification, not guaranteed on any printed maze. There is no motion safety guarantee from software alone.

## Quick PC test (Ubuntu 24.04, no ROS required)

```bash
cd robot_game_project
bash scripts/setup_ubuntu.sh
.venv/bin/python -m pytest -q
bash scripts/run_mock.sh --game 1
bash scripts/run_mock.sh --game 2 --seed 11
bash scripts/run_mock.sh --game 3
bash scripts/run_mock.sh --card-image assets/card_2.png
```

Open `out/tictactoe_result.png`, `out/maze_input.png` and `out/maze_solved.png`. Vary `--seed` and `--cells` to generate different mazes.

External maze images, once deskewed to a top-down rectangle and start/goal manually located:

```bash
bash scripts/run_mock.sh --game 2 --image /path/to/maze.png --start 35 35 --goal 440 440
```

Action card using a webcam (requires GUI and Tesseract OCR):

```bash
bash scripts/run_mock.sh --game camera
```

Camera flow recognizes **card** from a snapshot; for maze vision use a separate `--image` after choosing game; autonomous switching of camera FOV from card to board is not part of this prototype. OCR is English digits/labels by default; Korean action labels need `tesseract-ocr-kor` and language configuration.

## Native official Doosan controller emulator (ROS 2 Jazzy)

Follow the current official ROS 2 Jazzy guide (Ubuntu 24.04) and install ROS 2 Jazzy first:

https://doosanrobotics.github.io/doosan-robotics-ros-manual/jazzy/installation.html

Then:

```bash
bash scripts/setup_ros2_jazzy.sh
cd ~/ros2_ws/src/doosan-robot2
sudo ./install_emulator.sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=e0509
```

**Caveat**: `e0509` must exist in your checked-out Jazzy branch description/launch model list. Check first:

```bash
find ~/ros2_ws/src/doosan-robot2 -iname '*e0509*'
ros2 pkg list | grep dsr
```

If missing, do not silently replace it with another physical robot model. You may separately try a **supported model** such as `a0509` for studying the official emulator UI, but this does NOT validate E0509 geometry/limits. The mock simulator works without the vendor emulator. ROS Python imports and DSR runtime are from `~/ros2_ws/install/setup.bash`; the `.venv` may not inherit them. For commissioning run `/usr/bin/python3 -m robot_games.app ...` in a sourced terminal with the needed python modules installed for system Python, or use a ROS-accessible virtual environment created `--system-site-packages`.

## Actual robot — staged commissioning, no automatic first motion

1. Physical E-stop, risk assessment, restricted working area, enabling switch/teach mode, controller compatibility, firmware and tool safety verified by qualified technician. Never position people in reach while testing.
2. Check that `model:=e0509` and real-controller interface are supported by the precise Doosan Jazzy package/controller version. Verify actual controller IP on teach pendant; example launch only:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=e0509
ros2 node list
ros2 service list | grep dsr
```

3. Confirm network and state *without* motion. Start teach-pendant jog at reduced safe speed. Verify mounted tool/payload, TCP, pen, camera registration, board reachable volume, Z offset, force/contact mitigation, singularities and collision-free approach. This repository does not set TCP/robot modes or manage protective stops.
4. Before any real run: teach and validate 6 home joints, and distinct 6-joint **straight-arm celebration** position (it is **not** mathematically safe to assume straight means joint zero angles). Fill `configs/default.yaml` with these 6 joint angles, true robot-base board origin, base X and Y vectors (measured full-image span, 3 components each), absolute pen contact and lift heights, tool orientation, constrained workspace. **The current transform assumes a horizontal calibrated plane; an uncalibrated camera does not provide millimeter coordinates.**
5. Run mock first, set `robot.calibration_confirmed: true` only after coordinate/TCP verification, then set `robot.armed: true` with written local approval. Real execution has a third CLI interlock:

```bash
# THIS MOVES A PHYSICAL ROBOT IF PRECONDITIONS ARE MET. DO NOT RUN UNTIL COMMISSIONED.
python3 -m robot_games.app --mode real --execute-motion --config configs/default.yaml --game 2 --seed 7
```

`GuardedDoosan` uses `DSR_ROBOT2` `movej`, `movel` and assumes the driver/default motion reference frame, TCP, and robot state have been configured correctly. Confirm exact API semantics on your controller; no collision/force/pen-pressure controller is supplied. For initial verification use noncontact air-writing (contact Z above board) and teach-pendant speed controls. **No person should stand near the robot during movement.**

6. Gripper: RH-P12-RN(A) is 24 V RS-485, DYNAMIXEL protocol 2.0, model number 35074. Use suitable powered RS-485 interface, not raw PC serial. `dynamixel-sdk` driver is provided but disabled; verify USB serial permissions, ID, baud, mechanical stop positions, current limit, operating mode and firmware before use. **Do not send arbitrary open/close positions.** The separate gripper class is not wired to the game drawing motions because whether it holds a marker or cards has not been specified.

## Known constraints / next integrations

- Automatic camera-to-plane calibration: ArUco corner markers + homography + camera-to-robot transform.
- Tic-tac-toe: track human moves across frames, robust O/X classifier and game turn legality validation.
- Maze: automatically identify entrance/exit, distinguish printed path from walls, perspective correction and detect pen-contact errors.
- Add runtime supervisory stop, workspace limits, collision checks / MoveIt planning and recovery logic before unattended use.
- Motion driver requires actual E0509 controller validation; printing path points does NOT establish feasibility.

## Manufacturer references

- Doosan Jazzy: https://doosanrobotics.github.io/doosan-robotics-ros-manual/jazzy/installation.html
- Doosan virtual/real launch: https://doosanrobotics.github.io/doosan-robotics-ros-manual/jazzy/tutorials/basic_tutorials/rviz_tutorial.html
- DSR Python: https://doosanrobotics.github.io/doosan-robotics-ros-manual/humble/tutorials/advanced_tutorials/dsr_robot_tutorial.html
- ROBOTIS RH-P12-RN(A): https://emanual.robotis.com/docs/en/platform/rh_p12_rna/
