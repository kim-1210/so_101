# SO-101 follower를 MoveIt으로 움직이기

## 1. 빌드

작업공간에서 다음 명령을 실행합니다.

```bash
cd /media/kim/hdd1tb/master_study/so_101/so101_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select so101_moveit_config so101_bringup
source install/setup.bash
```

## 2. 하드웨어와 MoveIt 실행

실제 follower 포트에 맞춰 실행합니다.

```bash
ros2 launch so101_bringup moveit_follower.launch.py follower_port:=/dev/ttyUSB0
```

정상이라면 아래 액션도 보여야 합니다.

```bash
ros2 action list
# /so101_arm_controller/follow_joint_trajectory
```

## 3. 목표 6D pose 보내기

SO-101은 5축이므로 이 실행 파일은 기본적으로 `x, y, z` 위치만 목표로 사용합니다.
`roll, pitch, yaw`는 무시됩니다. 로그에 `Position-only target`이 표시되면 정상입니다.

입력은 `x, y, z, roll, pitch, yaw` 순서입니다. 위치 단위는 meter이고 회전은
기본적으로 radian입니다. `degrees:=true`를 넣으면 회전만 degree로 입력합니다.

먼저 `execute` 없이 경로를 만들 수 있는지 확인합니다.

```bash
ros2 run so101_moveit_config moveit_pose_goal --ros-args \
  -p x:=0.20 \
  -p y:=0.00 \
  -p z:=0.20 \
  -p roll:=0.0 \
  -p pitch:=0.0 \
  -p yaw:=0.0
```

정확한 자세까지 시험하려면 `-p position_only:=false`를 추가할 수 있지만, 5축으로
표현할 수 없는 6D 자세는 계획에 실패합니다.

계획 성공을 확인한 뒤 `-p execute:=true`를 추가하면 실제 로봇이 움직입니다.

```bash
ros2 run so101_moveit_config moveit_pose_goal --ros-args \
  -p degrees:=true \
  -p x:=0.20 \
  -p y:=0.00 \
  -p z:=0.20 \
  -p roll:=0.0 \
  -p pitch:=45.0 \
  -p yaw:=0.0 \
  -p execute:=true
```

좌표는 기본적으로 로봇의 `base` 기준입니다. SO-101 arm은 5축이므로 임의의 6D pose를
항상 정확히 만들 수는 없습니다. 입력한 pose를 만들 수 없으면 계획이 실패합니다.

로봇 팔이 닿지 않는 위치나 충돌하는 자세는 MoveIt이 거절합니다. 처음에는
`velocity_scale:=0.1`처럼 낮은 속도로 시험하세요.
