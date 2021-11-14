cmake_minimum_required(VERSION 3.5)
project(frootspi_wheel)

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
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(frootspi_wheel_component SHARED
  src/frootspi_wheel_component.cpp)
target_compile_definitions(frootspi_wheel_component
  PRIVATE "FROOTSPI_WHEEL_BUILDING_DLL")
ament_target_dependencies(frootspi_wheel_component
  frootspi_msgs
  rclcpp
  rclcpp_components
  geometry_msgs
)
rclcpp_components_register_nodes(frootspi_wheel_component "frootspi_wheel::WheelNode")

add_executable(frootspi_wheel_node src/main.cpp)
target_link_libraries(frootspi_wheel_node frootspi_wheel_component)
ament_target_dependencies(frootspi_wheel_node
  rclcpp)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(frootspi_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(geometry_msgs)

ament_export_include_directories(include)
ament_export_libraries(frootspi_wheel_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  frootspi_wheel_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS
  frootspi_wheel_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
