cmake_minimum_required(VERSION 3.8)
project(interactive_marker_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(basic_controls src/basic_controls.cpp)
ament_target_dependencies(basic_controls rclcpp interactive_markers tf2_geometry_msgs geometry_msgs tf2 tf2_ros)

target_include_directories(basic_controls PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(basic_controls PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS basic_controls
  DESTINATION lib/${PROJECT_NAME})
# pong game
add_executable(pong src/pong.cpp)
ament_target_dependencies(pong rclcpp interactive_markers tf2_geometry_msgs geometry_msgs tf2 tf2_ros)

target_include_directories(pong PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pong PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS pong
  DESTINATION lib/${PROJECT_NAME})
# end pong game

# pongPaddle
add_executable(pongpaddle src/pongpaddle.cpp)
ament_target_dependencies(pongpaddle rclcpp interactive_markers tf2_geometry_msgs geometry_msgs tf2 tf2_ros)

target_include_directories(pongpaddle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pongpaddle PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS pongpaddle
  DESTINATION lib/${PROJECT_NAME})
# end pongPaddle

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

ament_package()
