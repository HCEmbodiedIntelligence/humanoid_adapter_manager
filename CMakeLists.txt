cmake_minimum_required(VERSION 3.8)
project(humanoid_adapter_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME})

install(PROGRAMS
  scripts/humanoid_pluginctl.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_deployment_plugins
    test/test_deployment_plugins.py)
endif()

ament_package()
