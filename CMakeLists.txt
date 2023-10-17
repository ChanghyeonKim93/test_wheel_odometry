cmake_minimum_required(VERSION 3.10)
project(test_wheel_odometry)
message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME})

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -march=native")

find_package(Eigen3 REQUIRED)

include_directories ("/usr/include/opencv4/")
include_directories(${Eigen3_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/)

set(SRC ${SRC} 
  #src/numerical_integrator/runge_kutta_integrator_fourth_order.cc
)

# TEST
add_executable(${CMAKE_PROJECT_NAME}_test_runge_kutta test/test_runge_kutta.cc ${SRC})
target_link_libraries(${CMAKE_PROJECT_NAME}_test_runge_kutta ${Eigen3_LIBRARIES})