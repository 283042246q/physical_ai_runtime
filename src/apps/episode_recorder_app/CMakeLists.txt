cmake_minimum_required(VERSION 3.10)
project(episode_recorder_app)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/activate_lifecycle_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  package.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_app_contract test/test_app_contract.py)
endif()

ament_package()
