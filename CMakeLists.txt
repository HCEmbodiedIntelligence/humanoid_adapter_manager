cmake_minimum_required(VERSION 3.8)
project(humanoid_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME})

install(PROGRAMS
  scripts/humanoid_pluginctl.py
  scripts/configurator_launcher.py
  scripts/configuration_status.py
  scripts/ensure_controllers.py
  scripts/benchmark_alignment.py
  scripts/validate_session_export.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY docs launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY python/${PROJECT_NAME}/web/static
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/web)

install(FILES README.md requirements-web.txt
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_humanoid_manager
    test
    TIMEOUT 180
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()

ament_package()
