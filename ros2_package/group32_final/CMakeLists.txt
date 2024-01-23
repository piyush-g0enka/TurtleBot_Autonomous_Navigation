cmake_minimum_required(VERSION 3.8)
project(group32_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include/group32_final)

# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ros2_aruco_interfaces
  nav2_msgs
  mage_msgs
)
# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

#-----------------------------
# C++
#-----------------------------
add_executable(robot_move src/robot_move.cpp)
ament_target_dependencies(robot_move
  ${FRAME_DEMO_INCLUDE_DEPENDS}
)

add_executable(parts_broadcaster src/parts_broadcaster.cpp src/utils.cpp)
ament_target_dependencies(parts_broadcaster
  ${FRAME_DEMO_INCLUDE_DEPENDS}
)

add_executable(wp_publisher src/wp_publisher.cpp src/utils.cpp)
ament_target_dependencies(wp_publisher
  ${FRAME_DEMO_INCLUDE_DEPENDS}
)


#-----------------------------
# Installation
#-----------------------------

# Install the executable

install(TARGETS
  robot_move
  DESTINATION lib/${PROJECT_NAME}
)

# Install the executable
install(TARGETS
  parts_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# Install the executable
install(TARGETS
  wp_publisher
  DESTINATION lib/${PROJECT_NAME}
)
# Install the "config" folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install the "config" folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the "config" folder
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
