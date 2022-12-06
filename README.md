# unitree a1 motor with bullet3
bullet3의 내장 InverseDynamics 함수를 이용한 2-link manipulator의 중력보상 cpp 코드입니다.
빌드전 jsoncpp과 eigen library를 설치해야 합니다.
# jsoncpp install
```bash
  git clone https://github.com/open-source-parsers/jsoncpp.git
  cd  jsoncpp
  mkdir build
  cd build
  cmake ..
  make -j16
  sudo make install
```

# eigen3 install
```bash
   sudo apt-get install libeigen3-dev
   cd /usr/include 
   sudo ln -s eigen3/Eigen Eigen
```


# Install
```bash
  git clone https://github.com/tjdalsckd/a1_motor_cpp.git
  cd a1_mogor_cpp
  mkdir build
  cd build
  cmake ..
  make -j16
```

# Run
```bash
  bash start_pybullet_server.sh
```
```bash
  cd build
  ./pybullet_cpp_a1_motor 
```
