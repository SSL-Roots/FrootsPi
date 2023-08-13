cmake_minimum_required(VERSION 3.5)
project(frootspi_speaker)

# find dependencies
find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/speaker.py
    DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()