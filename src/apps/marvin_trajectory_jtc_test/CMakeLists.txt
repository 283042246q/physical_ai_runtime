cmake_minimum_required(VERSION 3.16)
project(marvin_trajectory_jtc_test)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/trajectory_chain_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
