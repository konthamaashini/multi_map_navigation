cmake_minimum_required(VERSION 3.8)
project(multi_map_navigation)

############################################################
#  Compiler flags
############################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############################################################
#  Dependencies
############################################################
find_package(ament_cmake REQUIRED)

# Core ROS packages you already listed
find_package(rclcpp          REQUIRED)
find_package(std_msgs        REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(nav_msgs        REQUIRED)
find_package(nav2_msgs       REQUIRED)

# TF / transforms
find_package(tf2             REQUIRED)
find_package(tf2_ros         REQUIRED)

# Gazebo + ros2_control
find_package(gazebo_ros      REQUIRED)          # launches Gazebo from ROS 2
find_package(controller_manager REQUIRED)
find_package(ros2_control    REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

# SQLite vendor (you had it; keep if you need database logging)
find_package(sqlite3_vendor  REQUIRED)

############################################################
#  (Optional) Add your C++ nodes here
############################################################
# file(GLOB SRC_FILES src/*.cpp)
# if(SRC_FILES)
#   add_executable(nav_node ${SRC_FILES})
#   ament_target_dependencies(nav_node
#     rclcpp geometry_msgs nav_msgs tf2_ros
#   )
#   install(TARGETS nav_node DESTINATION lib/${PROJECT_NAME})
# endif()

############################################################
#  Install resources
############################################################
install(
  DIRECTORY
    launch
    urdf
    worlds
    config         # ← controllers.yaml lives here
    rviz
    config
  DESTINATION share/${PROJECT_NAME}/
)

############################################################
#  Enable testing / linters
############################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

############################################################
#  Export package
############################################################
ament_package()
