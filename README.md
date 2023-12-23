# bullet3_robot_control
이 repository는 bullet3을 이용한 robot control 예제를 제공합니다.

- Joint Control
  - Computed Torque Control
  - Hinf Control
  - 추후 추가
- Task Control
  - Robust Control
  - 추후 추가


# install
실행하기 위해서 다음을 설치해야만 합니다.
```bash
sudo apt-get install libspdlog-dev
```
1. https://github.com/MinchangSung0223/LieGroupRoboticsControl.git

2. https://github.com/MinchangSung0223/SimRobot.git

```bash
  cd task_control/robust_control
  mkdir build
  cd build
  cmake ..
  make -j$(nproc)
  ./RobustControl
```
초록색 로봇 = 목표 위치
![image](https://github.com/MinchangSung0223/bullet3_robot_control/assets/53217819/24c8ce3e-8152-4658-9fd2-54287e34c969)

