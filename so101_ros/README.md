# ros2_control | plugin 진행 순서

```
[1] 순수 URDF
      ↓
[2] URDF에 <ros2_control> 작성
      ↓
[3] Mock Hardware 연결
      ↓
[4] controllers.yaml
      ↓
[5] ros2_control launch
      ↓
[6] Mock에서 controller 동작 검증
      ↓
-------------------------------
          가상 하드웨어
-------------------------------
      ↓
[7] Hardware PKG 생성
      ↓
[8] SystemInterface hpp/cpp 작성
      ↓
[9] Plugin XML 작성
      ↓
[10] CMakeLists + package.xml
      ↓
[11] Plugin 빌드
      ↓
[12] URDF의 Mock을 내 Plugin으로 교체
      ↓
[13] 가짜 read/write로 다시 검증
      ↓
-------------------------------
         Custom Plugin
-------------------------------
      ↓
[14] STS3215 통신 코드 추가
      ↓
[15] read() 실제값 읽기
      ↓
[16] write() 실제 명령 전송
      ↓
[17] 실제 로봇 제어
```

## ros2_control
### controllers.yaml 문법
```
- controller_manager: urdf에서 설정한 ros2_control에 관한 값들을 받음
    - ros__parameters: joint값을 읽거나 명령하기 위한 문법 정의
        - joint_state_broadcaster: joint값 외부 공개
            - type: joint_state_broadcaster/JointStateBroadcaster
        - arm_controller: joint값 명령 (한번에 가지않고 보간을 함)
            - type:joint_trajectory_controller/JointTrajectoryController
        - arm_controller: joint값 명령
            - type:position_controllers/JointGroupPositionController
```
### urdf에 설정방법
```
<ros2_control name="name" type="system">
    <hardware>
        # read나 write를 제공할 플러그인
        <plugin> mock_components/GenericSystem </plugin> ==> 가상설정
    </hardware>

    <joint name="joint name">
        <command_interface name="position | velocity | torque"> ==> 명령
        <state_interface name="position | velocity | torque"> ==> 상태
    </joint>
</ros2_control>
```
### launch 설정방법
```python
1. urdf를 읽어야함
robot_description = {
    "robot_description": Command(
        [
            "xacro ",
            xacro_file_path
        ]
    )
}

2. urdf의 컨트롤러 설정
controller_manager = Node(
    package="controller_manager",
    execute="ros2_control_node",
    parameters=[
        {"update_rate":50,},
        controllers_file # yaml파일
    ],
    remapping=[
        # 기본적인 내장은 ~/ 이지만 일반 /로 리맵핑
        ("~/robot_description", "/robot_description"),
    ],
    output="screen"
)

3. public state 설정
joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster", # joint_state_broadcaster라는 controller를 실행
        "--controller-manager", # 요청할 controller-manager 설정
        "/controller_manager"
    ],
    output="screen"
)

4. command 설정
arm_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        # 내가 설정한 이름
        "arm_controller",
        "--controller-manager",
        "/controller_manager"
    ],
    output="screen"
)
```
-----
-----
## plugin
### hardware PKG 설정
(write | read | init 관련)
- include -> ~.hpp : 부모 상속받고 클래스 선언
- src -> ~.cpp : 선언 내린 클래스 함수 정의

### 플러그인 설정 (.xml)
```xml
<library path="파일 위치">
    <class
        name="플러그인 이름"
        type="클레스 이름"
        base_class_type="type클레스의 상속 부모 클레스">

        <description>
            설명내용
        </description>

    </class>
</library>
```

### CMakeLists.xml 설정
```xml
# 공유할 라이브러리 설정
add_library(
  so101_hardware
  SHARED
  src/so101_hardware.cpp
)

# header 파일 경로 설정
target_include_directories(
  so101_hardware
  PUBLIC
    include
)

# so101_hardware를 컴파일할 때 이 ROS2 라이브러리들을 링크
ament_target_dependencies(
  so101_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# plugin XML을 ROS에 등록
pluginlib_export_plugin_description_file(
  hardware_interface
  so101_hardware.xml
)

install(
  TARGETS so101_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)
```