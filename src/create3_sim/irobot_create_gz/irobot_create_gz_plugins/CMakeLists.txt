cmake_minimum_required(VERSION 3.8)
project(irobot_create_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(OpenGL_GL_PREFERENCE LEGACY)

# Support newer gz package names and Humble-era ignition package names.
find_package(gz_gui_vendor QUIET)
if(gz_gui_vendor_FOUND)
  set(IROBOT_CREATE_GUI_VENDOR_DEP gz_gui_vendor)
else()
  find_package(ignition-gui6 REQUIRED)
  set(IROBOT_CREATE_GUI_VENDOR_DEP ignition-gui6)
endif()
add_subdirectory(Create3Hmi)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
