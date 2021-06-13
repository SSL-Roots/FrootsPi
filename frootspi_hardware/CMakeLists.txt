cmake_minimum_required(VERSION 3.5)
project(frootspi_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(frootspi_msgs REQUIRED)

include_directories(include)

add_library(driver_component SHARED
  src/driver_component.cpp)
target_compile_definitions(driver_component
  PRIVATE "FROOTSPI_HARDWARE_BUILDING_DLL")
ament_target_dependencies(driver_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  frootspi_msgs
)
target_link_libraries(driver_component pigpiod_if2)
rclcpp_components_register_nodes(driver_component "frootspi_hardware::Driver")

add_executable(driver_node src/hardware_driver.cpp)
target_link_libraries(driver_node driver_component)
ament_target_dependencies(driver_node
  rclcpp)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(std_msgs)
ament_export_dependencies(frootspi_msgs)

ament_export_include_directories(include)
ament_export_libraries(driver_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  driver_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
