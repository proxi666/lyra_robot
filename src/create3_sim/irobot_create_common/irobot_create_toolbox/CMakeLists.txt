cmake_minimum_required(VERSION 3.5)

project(irobot_create_toolbox)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_math_vendor QUIET)
if(gz_math_vendor_FOUND)
  find_package(gz-math REQUIRED)
  set(IROBOT_CREATE_MATH_TARGET gz-math::gz-math)
  set(IROBOT_CREATE_MATH_EXPORT_DEPS gz_math_vendor gz-math)
else()
  find_package(ignition-math6 REQUIRED)
  set(IROBOT_CREATE_MATH_TARGET ignition-math6::ignition-math6)
  set(IROBOT_CREATE_MATH_EXPORT_DEPS ignition-math6)
endif()
find_package(rclcpp REQUIRED)

#### Libraries

add_library(irobot_create_toolbox SHARED)
target_sources(
  irobot_create_toolbox
  PRIVATE
    src/polar_coordinates.cpp
)
target_include_directories(irobot_create_toolbox PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(irobot_create_toolbox
  PUBLIC
    ${IROBOT_CREATE_MATH_TARGET}
    ${rclcpp_TARGETS}
)

#### Install

install(TARGETS
  ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  ${IROBOT_CREATE_MATH_EXPORT_DEPS}
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
