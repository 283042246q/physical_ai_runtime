cmake_minimum_required(VERSION 3.16)
project(marvin_mpd_bimanual_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_description_contract
    test/test_description_contract.py
    TIMEOUT 60
  )
endif()

install(DIRECTORY
  config
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
