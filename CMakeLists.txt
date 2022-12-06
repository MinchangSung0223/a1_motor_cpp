cmake_minimum_required(VERSION 3.0)
FIND_PACKAGE(jsoncpp REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

include_directories(${JSONCPP_INCLUDE_DIRS})
include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    /usr/local/include
    ${JSONCPP_INCLUDE_DIRS}
)
link_directories(
    lib/
    ${JSONCPP_LIB_DIRS}
    /usr/local/lib
)

PROJECT(pybullet_cpp_indy7)

cmake_minimum_required(VERSION 3.12)

SUBDIRS(bullet3)
file(GLOB_RECURSE INDY7 src/Indy7.cpp src/Indy7.h )
file(GLOB_RECURSE MR lib/ModernRobotics.cpp lib/ModernRobotics.h)


add_executable(pybullet_cpp_a1_motor src/main.cpp ${INDY7} ${MR} )

target_link_libraries(pybullet_cpp_a1_motor PUBLIC pybullet_cpp)
target_link_libraries(pybullet_cpp_a1_motor PUBLIC jsoncpp_lib)
target_link_libraries(pybullet_cpp_a1_motor PUBLIC libunitreeMotorSDK_Linux64.so)
target_link_libraries(pybullet_cpp_a1_motor PUBLIC pthread)



