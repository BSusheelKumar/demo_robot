cmake_minimum_required(VERSION 3.8)
project(task_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)

# add_executable(map_lifecycle_manager src/load_map.cpp)
# ament_target_dependencies(map_lifecycle_manager rclcpp rclcpp_lifecycle)
# install(TARGETS
#   map_lifecycle_manager
#   DESTINATION lib/${PROJECT_NAME}
# )

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

install(
  DIRECTORY config description launch maps worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})


install(PROGRAMS
       scripts/map_manager.py
       scripts/cmd_vel_bridge.py
       scripts/lidar_based_stopping.py

       DESTINATION lib/${PROJECT_NAME}
)
ament_package()
