cmake_minimum_required(VERSION 3.5)

project(velodyne_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_lint_auto REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_ros_velodyne_laser SHARED src/GazeboRosVelodyneLaser.cpp)
ament_target_dependencies(gazebo_ros_velodyne_laser
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
)

add_library(gazebo_ros_velodyne_gpu_laser SHARED src/GazeboRosVelodyneLaser.cpp)
ament_target_dependencies(gazebo_ros_velodyne_gpu_laser
  "gazebo"
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
)
target_compile_definitions(gazebo_ros_velodyne_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)


ament_export_libraries(
  gazebo_ros_velodyne_laser 
  gazebo_ros_velodyne_gpu_laser
  )

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)

ament_lint_auto_find_test_dependencies()

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_velodyne_laser
    gazebo_ros_velodyne_gpu_laser
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
