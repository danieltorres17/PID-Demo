cmake_minimum_required(VERSION 3.5)
PROJECT(viconPose)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")

IF(NOT CMAKE_BUILD_TYPE)
# SET(CMAKE_BUILD_TYPE "Debug")
 SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

# Find HAL
find_package(HAL REQUIRED)
include_directories(${HAL_INCLUDE_DIRS})

# Find Eigen
find_package( Eigen3 REQUIRED)

add_executable( viconPose viconPose.cpp PID.cpp PID.h)
target_link_libraries(viconPose
  ${HAL_LIBRARIES}
  dl
)

