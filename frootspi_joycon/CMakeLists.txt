cmake_minimum_required(VERSION 3.5)
project(frootspi_joycon)

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
find_package(frootspi_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(joycon_component SHARED
  src/joycon_component.cpp)
target_compile_definitions(joycon_component
  PRIVATE "FROOTSPI_JOYCON_BUILDING_DLL")
ament_target_dependencies(joycon_component
  frootspi_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)
rclcpp_components_register_nodes(joycon_component "frootspi_joycon::JoyCon")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(frootspi_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rclcpp_lifecycle)

ament_export_include_directories(include)
ament_export_libraries(joycon_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  joycon_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
