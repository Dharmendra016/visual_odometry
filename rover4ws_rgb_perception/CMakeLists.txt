cmake_minimum_required(VERSION 3.8)
project(rover4ws_rgb_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/rgb_perception_node.py
  DESTINATION lib/${PROJECT_NAME})
# helper modules are imported from the script's own directory
install(FILES scripts/ground_geometry.py scripts/models.py scripts/freespace.py
              scripts/corridor.py scripts/flow.py scripts/tracker.py
  DESTINATION lib/${PROJECT_NAME})

# Files edited from Windows (\wsl.localhost) lose the executable bit; restore it on every build.
file(GLOB _scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
add_custom_target(${PROJECT_NAME}_chmod_scripts ALL COMMAND chmod +x ${_scripts})

ament_package()
