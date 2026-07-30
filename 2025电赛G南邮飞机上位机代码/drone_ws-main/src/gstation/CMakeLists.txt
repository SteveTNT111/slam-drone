cmake_minimum_required(VERSION 3.8)
project(gstation)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(msg_tool REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl REQUIRED)



add_executable(gstation src/GStation.cpp)

ament_target_dependencies(gstation 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  mavros_msgs 
  cv_bridge 
  OpenCV 
  msg_tool 
  tf2 
  tf2_geometry_msgs
  rcl
)

add_executable(serial src/SerialNode.cpp)
ament_target_dependencies(serial 
  rclcpp 
  std_msgs
  rcl
)

add_executable(landscreen src/LandScreen.cpp)
ament_target_dependencies(landscreen
  rclcpp
  std_msgs
  geometry_msgs
  rcl
  msg_tool
)

include_directories(
  include
        ${msg_tool_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
        ${tf2_geometry_msgs_INCLUDE_DIRS}
)
# 设置目标属性
set_target_properties(gstation PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
set_target_properties(serial PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ament_target_dependencies(gstation rclcpp std_msgs mavros_msgs geometry_msgs sensor_msgs cv_bridge OpenCV nav_msgs msg_tool)
# ament_target_dependencies(serial rclcpp std_msgs mavros_msgs msg_tool geometry_msgs)

# 安装规则
install(TARGETS
  gstation
  serial
  landscreen
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出依赖
ament_export_dependencies(
  rclcpp
  rcl
  std_msgs
  mavros_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  opencv2
  nav_msgs
  msg_tool
)

ament_package()
