# Berlin 장애물 4개 A/B 재현

## 실험 설정

- Docker 컨테이너 `adp_sim`, 사용한 로컬 이미지 태그 `misys:forza_full`, ROS 2 Humble
- MISYS Forza 스택의 소스 커밋 `bfba899165e4de371ac4014cad52add06f27559d`
- Berlin 맵의 `speed_scaling.yaml`에서 A/B 모두 `global_limit: 0.50` 사용. 이 값은 전역 속도 프로파일을 스케일하는 계수입니다.
- 두 조건 모두 기존 C++ MAP 조향기, `SIM_linear` LUT, 스플라인 회피, 시뮬레이터 ground-truth 위치·속도를 사용했습니다.
- A는 MAP의 출력 속도를 그대로 시뮬레이터에 전달합니다. B는 같은 출력에 로컬 경로 앞쪽 2.0 m의 곡률을 이용한 속도 상한을 추가합니다. 횡가속과 감속 한계 계수는 각각 2.5 m/s²입니다. 기존 제어기나 플래너의 소스를 수정하거나 ROS 패키지를 새로 빌드하지 않았습니다.
- 최근에 제안한 회피 진입·복귀 거리 조정은 후속 실험 아이디어입니다. 이 ZIP은 완료한 곡률 감속 A/B 실험을 재현합니다.

## 파일

- `berlin_ab/berlin_trial.launch.py`는 시뮬레이터, 기존 주행 노드, 장애물 생성기를 실행합니다.
- `berlin_ab/run_trials.py`는 A/B 전환과 속도 상한 적용을 담당하며 랩타임·충돌·장애물 좌표를 JSONL에 저장합니다.
- `berlin_ab/obstacle_config*.yaml`은 기본, 시케인 설정, 레이스라인 차단 배치에 사용한 설정입니다.
- `berlin_ab/installed_spawner_filter.patch`는 실험 전부터 설치본에 있던 장애물 필터와 저장소 원본 간의 차이입니다. 기본·시케인 설정에 사용한 기존 필터를 임시 복사본에서 재현할 때 씁니다.
- `berlin_analysis/`에는 실제로 사용한 Berlin 지도 이미지, 맵 YAML, 전역 웨이포인트, 속도 설정이 들어 있습니다.
- `berlin_ab/*runs.jsonl`에는 기존 실측 결과가 들어 있습니다. 재실행 결과는 아래 명령에서 `replay/`에 저장됩니다.

## ROS 의존성

기존 `install/setup.bash`로 로드한 패키지는 `stack_master`, `f1tenth_gym_ros`, `global_planner`, `speed_scaler`, `frenet_odom_republisher`, `state_machine`, `controller_cpp`, `spline_planner`, `perception`, `perception_cpp`, `race_manager`입니다. 실행 스크립트는 설치된 `ackermann_msgs`와 `f110_msgs` 메시지도 사용합니다. ZIP에는 Docker 이미지와 ROS 빌드 산출물이 포함되어 있지 않습니다. 같은 버전의 시뮬레이터와 장애물 입력·충돌 토픽을 지원하는 MISYS 스택을 준비해 주세요.

## 실행

ZIP을 풀어서 `README.md`가 있는 폴더를 컨테이너의 `/tmp/berlin_ab_share`에 복사한 뒤 컨테이너의 Bash에서 실행합니다. 컨테이너 경로가 다르면 `STACK`과 `EXP`를 맞춰 주세요.

```bash
STACK="$HOME/forza_ws/race_stack"
EXP=/tmp/berlin_ab_share
source /opt/ros/humble/setup.bash
source "$STACK/install/setup.bash"
export ROS_DOMAIN_ID=72 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export AB_MAP_DIR="$EXP/berlin_analysis"
```

### 기본 배치와 시케인 설정

원본 `race_manager`를 임시 폴더에 복사하고 기존 설치본에 있던 후보 필터를 적용합니다. 본 실험에서는 동일한 필터가 이미 적용된 설치본을 사용했습니다. 시케인 설정에서도 이 필터 때문에 생성기가 난이도 조건을 완화했습니다.

```bash
FILTERED=$(mktemp -d)
cp -a "$STACK/utilities/nodes/race_manager/race_manager" "$FILTERED/"
patch -d "$FILTERED" -p1 < "$EXP/berlin_ab/installed_spawner_filter.patch"
export AB_RACE_MANAGER_SOURCE="$FILTERED"

AB_OBSTACLE_CONFIG="$EXP/berlin_ab/obstacle_config.yaml" \
  python3 "$EXP/berlin_ab/run_trials.py" --seeds 2026 2027 2028 2029 2030 2031 \
  --out "$EXP/replay/basic"

AB_OBSTACLE_CONFIG="$EXP/berlin_ab/obstacle_config_chicane.yaml" \
  python3 "$EXP/berlin_ab/run_trials.py" --seeds 2026 2029 \
  --out "$EXP/replay/chicane"
```

첫 확인만 하려면 기본 배치 명령의 시드를 `--seeds 2026`으로 줄이면 됩니다.

### 레이스라인을 가로막는 배치

이 배치에서는 필터가 없는 저장소 원본 장애물 생성기를 사용합니다.

```bash
AB_RACE_MANAGER_SOURCE="$STACK/utilities/nodes/race_manager" \
AB_OBSTACLE_CONFIG="$EXP/berlin_ab/obstacle_config_center.yaml" \
  python3 "$EXP/berlin_ab/run_trials.py" --seeds 2026 \
  --out "$EXP/replay/center"
```

각 시드에서 A와 B가 같은 장애물 좌표와 방향을 사용합니다. 랩타임은 시뮬레이터의 ground-truth odometry 타임스탬프로 계측한 벽시계 시간이며, 환경과 부하에 따라 달라질 수 있습니다.

