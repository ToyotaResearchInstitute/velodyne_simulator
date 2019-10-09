cmake_minimum_required(VERSION 3.7.2)

project(velodyne_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we don't use add_compile_options with pedantic in message packages
  # because the Python C extensions don't comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include
${gazebo_dev_INCLUDE_DIRS}
${gazebo_ros_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${tf2_INCLUDE_DIRS}
)
link_directories(${gazebo_dev_LIBRARY_DIRS})

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined; have you sourced your ROS distro setup.bash?")
endif()
string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
add_definitions(-DROS_${ROS_DISTRO})


add_library(gazebo_ros_velodyne_laser SHARED src/GazeboRosVelodyneLaser.cpp)
ament_target_dependencies(gazebo_ros_velodyne_laser
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
)
ament_export_libraries(gazebo_ros_velodyne_laser)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_velodyne_laser
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
