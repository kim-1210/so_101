# so101_moveit_test

PlanningScene에 박스 장애물을 넣고, MoveIt이 그 장애물을 고려해서 SO-101 arm 경로를
계획하는 테스트 패키지입니다.

## 빌드

```bash
cd /media/kim/hdd1tb2/master_study/so_101/so101_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select so101_moveit_test
source install/setup.bash
```

## RViz fake controller 데모

아래 명령은 `so101_moveit_config`의 demo launch를 같이 띄우고, 5초 뒤 장애물/계획
노드를 실행합니다. 기본값은 fake controller에서 실행까지 합니다.

```bash
ros2 launch so101_moveit_test planning_scene_obstacle_demo.launch.py
```

움직이지 않고 경로와 장애물만 보고 싶으면:

```bash
ros2 launch so101_moveit_test planning_scene_obstacle_demo.launch.py execute:=false
```

`execute:=false`이면 실제 로봇이나 fake controller는 움직이지 않습니다. 대신 노드가
`/display_planned_path`로 계획 경로를 보내서 RViz의 Planned Path 애니메이션으로
보여줍니다. RViz에서 바로 꺼지는 것처럼 보이면 `hold_seconds:=15.0`처럼 늘려보세요.

## 이미 MoveIt을 띄운 상태에서 노드만 실행

실제 하드웨어를 쓸 때는 먼저 `execute:=false`로 계획만 확인하세요.

```bash
ros2 launch so101_moveit_test planning_scene_obstacle_node.launch.py execute:=false
```

확인 후 실제 실행:

```bash
ros2 launch so101_moveit_test planning_scene_obstacle_node.launch.py execute:=true
```

## 자주 바꾸는 파라미터

기본 목표는 IK 실패를 줄이기 위해 joint target입니다.

```bash
ros2 launch so101_moveit_test planning_scene_obstacle_node.launch.py \
  joint_rotation:=-0.7 \
  joint_pitch:=0.6 \
  joint_elbow:=-0.8 \
  obstacle_x:=0.20 \
  obstacle_y:=-0.18 \
  obstacle_z:=0.10 \
  obstacle_size_x:=0.08 \
  obstacle_size_y:=0.14 \
  obstacle_size_z:=0.16
```

손끝 위치 목표를 쓰고 싶으면 `target_type:=pose`로 바꾸고 `goal_x`, `goal_y`,
`goal_z`를 조정하세요. SO-101은 5축이라 모든 위치/자세가 항상 되는 것은 아닙니다.
