cmake_minimum_required(VERSION 3.8)
project(stewart_platform)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo_ros REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_msgs
  gazebo_ros
)

# Install the Python package into site-packages
ament_python_install_package(${PROJECT_NAME})

#Install entry-point for python scripts
install(PROGRAMS
  scripts/controller_platform_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(ik src/ik.cpp)
ament_target_dependencies(ik ${dependencies})
target_link_libraries(ik Eigen3::Eigen)
target_include_directories(ik PRIVATE ${EIGEN3_INCLUDE_DIRS})
install(TARGETS ik
  DESTINATION lib/${PROJECT_NAME})

add_library(stewart_joint_controller SHARED src/plugins/joint_controller.cpp)
ament_target_dependencies(stewart_joint_controller ${dependencies})
target_include_directories(stewart_joint_controller PRIVATE
  ${EIGEN3_INCLUDE_DIRS})
install(TARGETS stewart_joint_controller
  DESTINATION lib)

install(DIRECTORY
  config
  launch
  stewart
  stewart_sdf_model
  world
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies} Eigen3 gazebo_ros gazebo_msgs)
ament_package()
