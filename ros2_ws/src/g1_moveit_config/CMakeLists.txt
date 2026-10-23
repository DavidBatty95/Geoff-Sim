cmake_minimum_required(VERSION 3.8)
project(g1_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md NOTICE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py
    ENV
      G1_MOVEIT_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config
      G1_MOVEIT_URDF=${CMAKE_CURRENT_SOURCE_DIR}/../unitree_g1_description/urdf/g1_29dof_dex3.urdf
  )
endif()

ament_package()
