# bullet3_robot_control
이 repository는 bullet3을 이용한 robot control 예제를 제공합니다.

- Joint Control
  - Computed Torque Control
  - Hinf Control
  - 추후 추가
- Task Control
  - Robust Control
  - 추후 추가



# 설치 방법 상세 설명
1. wsl 설치 (Windows11인 경우만) [설치방법](https://learn.microsoft.com/ko-kr/windows/wsl/install)
2. Ubuntu 20.04.6 설치
3. 기본 라이브러리 설치
   
```bash
   sudo apt-get update
   sudo apt-get install -y build-essential cmake git
   sudo apt-get install libeigen3-dev liburdfdom-dev
```
5. robotpkg install
   
pinocchio를 사용하기 위한 라이브러리입니다.
http://robotpkg.openrobots.org/debian.html
```bash
  sudo mkdir -p /etc/apt/keyrings
  
  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc |
     sudo tee /etc/apt/keyrings/robotpkg.asc
  
  sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
  deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub focal robotpkg
  EOF
```

6. pinocchio install
   
pinocchio는 Robot Dynamics 해석용 라이브러리입니다. M, C, G를 계산합니다.

https://stack-of-tasks.github.io/pinocchio/download.html
```bash
   sudo apt install -qqy lsb-release curl
   sudo mkdir -p /etc/apt/keyrings
   curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
       | sudo tee /etc/apt/keyrings/robotpkg.asc
   echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
       | sudo tee /etc/apt/sources.list.d/robotpkg.list
   sudo apt update
   sudo apt install -qqy robotpkg-py3*-pinocchio
   echo 'export PATH=/opt/openrobots/bin:$PATH
         export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
         export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
         export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH # Adapt your desired python version here
         export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> ~/.bashrc
```
7. ruckig install
   
ruckig는 Scurve Trajectory 생성용 라이브러리입니다. 
```bash
   git clone https://github.com/pantor/ruckig.git
   cd ruckig
   export RUCKIG_HOME=$PWD
   mkdir build
   cd build
   cmake ..
   make -j$(nproc)
   sudo make install
   cd /opt
   sudo  ln -s $RUCKIG_HOME .
```
8. LieGroupRoboticsControl install

Controller와 LieGroup연산을 수행하는 라이브러리입니다.
```bash
  git clone https://github.com/MinchangSung0223/LieGroupRoboticsControl.git
  cd LieGroupRoboticsControl
  mkdir build
  cd build
  cmake ..
  make -j$(nproc)
  sudo make install
```

10. SimRobot install

Bullet3용 Simulation Robot을 설정하는 라이브러리입니다.
```bash
  git clone https://github.com/MinchangSung0223/SimRobot.git
  cd SimRobot
  mkdir build
  cd build
  cmake ..
  make -j$(nproc)
  sudo make install
```
11. bullet3_robot_control 설치
실행하기 위해서 다음을 설치해야만 합니다.

```bash
  mkdir build
  cd build
  cmake ..
  make -j$(nproc)
```
초록색 로봇 = 목표 위치
![image](https://github.com/MinchangSung0223/bullet3_robot_control/assets/53217819/24c8ce3e-8152-4658-9fd2-54287e34c969)

