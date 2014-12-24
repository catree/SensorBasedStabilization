cmake_minimum_required(VERSION 2.8.4)
project(SensorBasedStabilization)

find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp VideoStabilization.cpp Quaternion.cpp MathUtil.cpp EulerAngles.cpp Matrix4x3.cpp
RotationMatrix.cpp vector3.h test.cpp)

add_executable(SensorBasedStabilization ${SOURCE_FILES})

target_link_libraries( SensorBasedStabilization ${OpenCV_LIBS} )