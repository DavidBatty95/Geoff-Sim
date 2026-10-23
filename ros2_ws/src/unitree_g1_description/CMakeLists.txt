cmake_minimum_required(VERSION 3.8)
project(unitree_g1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes urdf dex3_sources
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_description_files test/test_description_files.py)
endif()

ament_package()
