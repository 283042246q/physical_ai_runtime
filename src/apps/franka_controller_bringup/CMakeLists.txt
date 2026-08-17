cmake_minimum_required(VERSION 3.16)
project(franka_controller_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
