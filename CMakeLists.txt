cmake_minimum_required(VERSION 3.5)
project(cleaner_robot_fyp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(image_proc REQUIRED)

# ── Docking: detected_dock_pose_publisher ──────────────────────────────────
add_executable(detected_dock_pose_publisher
  src/detected_dock_pose_publisher.cpp
)
ament_target_dependencies(detected_dock_pose_publisher
  rclcpp
  geometry_msgs
  tf2_ros
)

install(TARGETS
  detected_dock_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)
# ──────────────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds src models
  DESTINATION share/${PROJECT_NAME}
)

# ── Python exploration scripts ─────────────────────────────────────────────
install(
  PROGRAMS
    scripts/wavefront_frontier_exploration.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wavefront_frontier_exploration
)

install(
  PROGRAMS
    scripts/map_ready_guard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_ready_guard
)

install(
  PROGRAMS
    scripts/nav_ready_guard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav_ready_guard
)

install(
  PROGRAMS
    scripts/ipa_coverage_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ipa_coverage_client
)

install(
  PROGRAMS
    scripts/coverage_path_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_path_executor
)

install(
  PROGRAMS
    scripts/coverage_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_manager
)

install(
  PROGRAMS
    scripts/coverage_progress_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_progress_tracker
)
# ───────────────────────────────────────────────────────────────────────────

ament_package()
