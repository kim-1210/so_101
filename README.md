# SO-101 로봇 제어 · 비전 · 시뮬레이션

SO-101 로봇 팔을 **ROS 2로 제어**하고, **RealSense 카메라로 물체 위치를 찾고**, **Isaac Lab에서 집기·놓기 동작을 실험**하는 연구용 작업공간입니다.

음성 명령에서 집을 물체와 놓을 장소를 찾는 실험 코드도 포함합니다. 각 기능을 따로 실행하는 구조이며, 모든 기능을 한 번에 시작하는 통합 실행 파일은 없습니다.

## 주요 기능

- **실제 로봇 제어:** STS 모터 통신, leader/follower 연동, `ros2_control` 하드웨어 플러그인.
- **경로 계획:** MoveIt 설정, RViz 시각화, 장애물이 있는 환경에서의 경로 계획 실험.
- **물체 인식:** SAM3 및 RF-DETR 관련 코드, 깊이 영상으로 계산한 물체 위치·자세 발행.
- **음성 입력:** Whisper 음성 인식과 LLM을 이용한 물체·목적지 추출.
- **시뮬레이션:** Isaac Lab의 Jacobian 기반 역기구학과 실제 로봇 관절 동기화.

대표적인 데이터 흐름은 다음과 같습니다.

```text
마이크 → Whisper → LLM → 집을 물체 / 놓을 장소
                              ↓
RealSense RGB·깊이 영상 → SAM3 → 로봇 기준 물체 좌표
                              ↓
                     ROS 2 물체·목표 토픽
                              ↓
                    Isaac Lab 동작 계산
                              ↓
                    ROS 2 follower 제어
```

## 폴더 구성

| 경로 | 역할 |
| --- | --- |
| `base_obj_tf/` | 물체 인식, 카메라 좌표 변환, ROS 2 목표 위치 발행 |
| `base_obj_tf/llm_ai/` | GPT / Gemini를 이용한 명령 해석 |
| `base_obj_tf/speak_to_sam/` | 마이크 녹음과 음성 인식 |
| `isaac_ros_so/` | Isaac Lab 시뮬레이션과 ROS 2 연결 |
| `isaac_ros_so/isaac_jacobian_2/` | 집기·놓기 제어를 모듈별로 나눈 코드 |
| `so101_ros/src/so101_hardware/` | 모터 통신 및 하드웨어 인터페이스 |
| `so101_ros/src/so101_bringup/` | 로봇 실행 설정과 관절 명령 연결 |
| `so101_ros/src/so101_moveit_config/` | MoveIt 및 컨트롤러 설정 |
| `so101_ros/src/so101_moveit_test/` | 장애물 추가와 경로 계획 데모 |
| `so101_ros/src/so_arm_description/` | URDF, 메시, 로봇 시각화 자료 |
| `camera_on.py`, `webcam_on.py` | 카메라 연결 확인 |
| `check_motor.py`, `register_sts_motor.py` | 모터 연결 확인 및 등록 도구 |
| `pyproject.toml`, `uv.lock` | Python 의존성과 버전 기록 |

## 준비 환경

- ROS 2 Humble, `colcon`, `rosdep`을 사용할 수 있는 Linux 환경.
- Python 3.10: `.python-version` 기준. ROS 2 Python 모듈과 같은 버전으로 맞춥니다.
- Python 패키지 관리 도구 `uv`.
- 실제 장비 실험 시 SO-101, 시리얼 어댑터, Intel RealSense 카메라. 음성 입력에는 마이크가 필요합니다.
- 시뮬레이션에는 별도로 구성한 Isaac Sim / Isaac Lab 환경이 필요합니다. 이 환경은 `uv sync`만으로 설치되지 않습니다.
- `pyproject.toml`은 PyTorch 2.8.0의 CUDA 12.9 배포 경로를 사용합니다. GPU와 드라이버 환경에 맞는지 확인하세요.

## 시작하기

### 1. 저장소 받기

```bash
git clone https://github.com/kim-1210/so_101.git
cd so_101
```

비공개 저장소이므로 접근 권한이 있는 GitHub 계정이 필요합니다.

### 2. Python 환경 준비

새로 받은 작업공간에서 실행합니다. ROS 2 Humble이 사용하는 시스템 Python이 3.10인지 먼저 확인하세요.

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 --version
uv venv --python /usr/bin/python3 --system-site-packages .venv
uv sync --locked
source .venv/bin/activate
python -c "import rclpy; print('ROS 2 Python 연결 확인')"
```

Python 패키지는 프로젝트의 `.venv`에 설치합니다. 글로벌 `pip install`은 사용하지 않습니다. Isaac Lab용 Python 환경은 별도로 관리합니다.

### 3. 로컬 전용 파일 준비

아래 파일은 GitHub에 포함하지 않습니다. 사용할 기능에 맞춰 본인 장비의 파일을 준비해야 합니다.

| 로컬 파일 | 필요한 기능 |
| --- | --- |
| `base_obj_tf/base_camera.json` | 카메라에서 로봇 기준 좌표로 변환. `position`은 3개 값, `orientation`은 `[x, y, z, w]` 쿼터니언 |
| `so101_ros/src/so101_hardware/config/so101_follower.json` | follower 모터별 보정값 |
| `so101_ros/src/so101_hardware/config/so101_leader.json` | leader를 사용할 때의 모터별 보정값 |
| `base_obj_tf/llm_ai/api_key.json` | LLM을 호출하는 음성 명령 처리 |
| `base_obj_tf/best_model.pt` | RF-DETR 기반 `ObjectSegmenter`를 사용하는 실험 |

API 키 파일은 다음 예제를 복사한 뒤 본인 키를 입력합니다. 이미 파일이 있다면 덮어쓰지 마세요.

```bash
cp -n base_obj_tf/llm_ai/api_key.example.json base_obj_tf/llm_ai/api_key.json
```

`gpt_key`와 `gemini_key` 중 사용하는 서비스의 항목을 채웁니다. 실제 키 파일은 `.gitignore`로 제외됩니다. SAM3 등 외부 모델의 다운로드와 접근 권한은 별도로 준비합니다.

카메라·모터 보정 코드와 측정 자료도 이 저장소의 업로드 범위에서 제외했습니다. 실제 로봇을 사용할 때는 장비에 맞는 보정값을 별도로 준비하세요.

### 4. ROS 2 패키지 빌드

ROS 빌드는 가상환경을 활성화하지 않은 새 터미널에서 진행합니다. `rosdep`은 시스템에서 초기 설정을 마친 상태를 가정합니다.

```bash
cd so101_ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

실제 장비용 JSON은 빌드 전에 `so101_hardware/config/`에 준비하면 설치 경로에 함께 반영됩니다. 이후 새 터미널에서도 ROS 2와 작업공간의 `setup.bash`를 불러와야 합니다.

## 실행 예시

### MoveIt / RViz 경로 계획

ROS 2 작업공간을 불러온 터미널에서 실행합니다.

```bash
ros2 launch so101_moveit_test planning_scene_obstacle_demo.launch.py execute:=false
```

가상 환경에 장애물을 추가하고 계획 경로를 표시합니다. 세부 옵션은 [MoveIt 장애물 데모](so101_ros/src/so101_moveit_test/README.md)를 참고하세요.

### 실제 follower 연결

포트와 보정 파일을 본인 장비에 맞게 지정합니다.

```bash
ros2 launch so101_bringup moveit_follower.launch.py \
  follower_port:=/dev/ttyUSB0 \
  follower_config_path:=/absolute/path/to/so101_follower.json
```

실제 로봇을 제어하는 실행입니다. 자세한 연결 절차는 [MoveIt follower 안내](so101_ros/src/so101_bringup/MOVEIT_FOLLOWER.md)에 있습니다.

### 카메라 물체 인식 / 음성 명령

프로젝트 루트에서 ROS 2 환경과 `.venv`를 불러온 뒤, 사용할 실행 파일 하나를 선택합니다.

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate

# 카메라 기반 물체 인식
python base_obj_tf/base_obj_tf.py --visual true

# 음성으로 물체와 목적지를 지정하는 실험
python base_obj_tf/base_obj_tf_2.py --visual true
```

두 실행 파일은 같은 카메라를 사용하므로 각각 실행합니다. 음성 버전은 검출 결과를 확인하는 GUI 창을 사용하므로 화면 환경이 필요합니다.

### Isaac Lab 시뮬레이션

Isaac Lab 환경에서 실행하며, ROS 2 모듈을 불러올 수 있어야 합니다. 현재 `isaac_ros_so/robot/so_arm.py`의 `usd_path`에는 개발 PC의 절대 경로가 들어 있으므로, 먼저 본인의 `isaac_ros_so/robot/so_arm.usd` 위치로 바꿉니다.

프로젝트 루트에서 Isaac Lab 설치 위치에 맞춰 실행합니다.

```bash
export PYTHONPATH="$PWD/isaac_ros_so${PYTHONPATH:+:$PYTHONPATH}"
/path/to/IsaacLab/isaaclab.sh -p isaac_ros_so/isaac_jacobian_2/main.py \
  --num_envs 1 \
  --sim_to_real false
```

이 예시는 실제 로봇 동기화를 끈 상태입니다. 집기·놓기에는 물체 인식 노드가 보내는 목표 토픽이 필요합니다. `--sim_to_real true`는 실제 관절 명령을 보내므로 장비 연결과 보정값을 준비한 뒤 사용합니다.

## 주요 ROS 2 토픽

| 토픽 | 메시지 타입 | 내용 |
| --- | --- | --- |
| `/obj_bases` | `geometry_msgs/msg/PoseArray` | 로봇 기준 물체 위치와 자세 |
| `/gripper_targets` | `geometry_msgs/msg/PoseArray` | 그리퍼가 향할 목표 위치와 자세 |
| `/basket` | `geometry_msgs/msg/Pose` | 물체를 놓을 장소 |
| `/follower/joint_states` | `sensor_msgs/msg/JointState` | 실제 follower 관절 상태 |
| `/follower/joint_commands` | `std_msgs/msg/Float64MultiArray` | follower에 보낼 관절 명령 |

## 업로드 제외 기준

[`.gitignore`](.gitignore)에 다음 항목을 정리했습니다. 원본 파일은 로컬에 유지합니다.

- ChArUco, checkerboard, calibration 관련 코드·이미지·측정 자료와 `calc_intrinsic/`.
- 카메라 변환값과 장비별 leader/follower 모터 보정 JSON.
- 실제 API 키, `.env`, 인증서와 개인 키.
- `.venv`, Python 캐시, ROS 2의 `build/`, `install/`, `log/`, 시뮬레이터 캐시.
- 모델 가중치, 녹음, 디버그 이미지, 실험 노트북, 발표 자료.
- 미완성 `smolvla/`·데이터셋 저장 코드, 사용하지 않는 RB5 모델, 로컬 SSH 탐색 도구.
- 개인 편집기 설정과 임시 파일.

소스 코드, 실행 설정, `uv.lock`, SO-101의 URDF·STL·USD 모델은 포함합니다. 새로 받은 저장소에는 보정값과 모델 가중치가 없으므로, 모든 실험이 바로 실행되는 형태는 아닙니다. 일부 기존 실행 파일과 하위 문서에는 개발 PC 경로가 남아 있어 본인 환경에 맞춰야 합니다.

## 추가 문서

- [ROS 2 하드웨어 플러그인 개발 메모](so101_ros/README.md)
- [MoveIt follower 실행 안내](so101_ros/src/so101_bringup/MOVEIT_FOLLOWER.md)
- [장애물 경로 계획 데모](so101_ros/src/so101_moveit_test/README.md)
- [Isaac Jacobian 코드 구성](isaac_ros_so/isaac_jacobian_2/README.md)
