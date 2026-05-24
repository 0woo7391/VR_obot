cmake_minimum_required(VERSION 3.8)
project(vrobot_command)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vr_command_handler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
