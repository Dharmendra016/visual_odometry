cmake_minimum_required(VERSION 3.8)
project(rover4ws_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME})

# Files edited from Windows (\wsl.localhost) lose the executable bit; restore it on every build.
file(GLOB _scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
add_custom_target(${PROJECT_NAME}_chmod_scripts ALL COMMAND chmod +x ${_scripts})

ament_package()
