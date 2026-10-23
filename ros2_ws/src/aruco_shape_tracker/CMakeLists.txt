cmake_minimum_required(VERSION 3.16)
project(aruco_shape_tracker)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs aruco)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(aruco_shape_tracker_cpp src/tracker_node.cpp)
target_compile_features(aruco_shape_tracker_cpp PRIVATE cxx_std_17)
target_link_libraries(aruco_shape_tracker_cpp
  Eigen3::Eigen
  ${OpenCV_LIBS}
  realsense2::realsense2
)
ament_target_dependencies(aruco_shape_tracker_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS aruco_shape_tracker_cpp
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_python_tests test)
endif()

ament_package()
