cmake_minimum_required(VERSION 3.16)
project(franka_motion_demos)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_demo_contract test/test_demo_contract.py)
endif()

install(PROGRAMS
  scripts/move_to_start.py
  scripts/send_mpd_trajectory.py
  scripts/smooth_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES
  README.md
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
