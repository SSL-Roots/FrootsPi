cmake_minimum_required(VERSION 3.5)
project(frootspi_conductor)

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
find_package(consai_frootspi_msgs REQUIRED)
find_package(frootspi_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(conductor_component SHARED
  src/conductor_component.cpp)
target_compile_definitions(conductor_component
  PRIVATE "FROOTSPI_CONDUCTOR_BUILDING_DLL")
ament_target_dependencies(conductor_component
  consai_frootspi_msgs
  frootspi_msgs
  rclcpp
  rclcpp_components
  geometry_msgs
  std_srvs
)
rclcpp_components_register_nodes(conductor_component "frootspi_conductor::Conductor")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(consai_frootspi_msgs)
ament_export_dependencies(frootspi_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(std_srvs)

ament_export_include_directories(include)
ament_export_libraries(conductor_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  conductor_component
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
