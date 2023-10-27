cmake_minimum_required(VERSION 3.10)
project(test_wheel_odometry)
message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME})

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -march=native")

find_package(Eigen3 REQUIRED)

include_directories ("/usr/include/opencv4/")
include_directories(${Eigen3_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/)
include_directories("/usr/include/python3.8")

set(SRC ${SRC} 
)

add_executable(test_runge_kutta test/test_runge_kutta.cc ${SRC})
target_link_libraries(test_runge_kutta ${Eigen3_LIBRARIES})

add_executable(test_data_generation test/test_data_generation.cc ${SRC})
target_link_libraries(test_data_generation ${Eigen3_LIBRARIES} python3.8)

add_executable(test_data_plot test/test_data_plot.cc ${SRC})
target_link_libraries(test_data_plot ${Eigen3_LIBRARIES} python3.8)

add_executable(test_kf test/test_kf.cc ${SRC})
target_link_libraries(test_kf ${Eigen3_LIBRARIES} python3.8)

add_executable(test_kf_with_acc test/test_kf_with_acc.cc ${SRC})
target_link_libraries(test_kf_with_acc ${Eigen3_LIBRARIES} python3.8)

add_executable(test_eskf test/test_eskf.cc 
  wheel_imu_error_state_kalman_filter/wheel_imu_error_state_kalman_filter.cc 
  ${SRC})
target_link_libraries(test_eskf ${Eigen3_LIBRARIES} python3.8)

add_executable(test_eskf2 test/test_eskf2.cc 
  wheel_imu_error_state_kalman_filter/wheel_imu_error_state_kalman_filter.cc 
  ${SRC})
target_link_libraries(test_eskf2 ${Eigen3_LIBRARIES} python3.8)

