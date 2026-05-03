cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)

find_package(gz_gui_vendor QUIET)
if(gz_gui_vendor_FOUND)
  find_package(gz-gui REQUIRED)
  set(IROBOT_CREATE_GUI_TARGET gz-gui::gz-gui)
  set(IROBOT_CREATE_GUI_AMENT_DEPS gz_gui_vendor gz-gui)
else()
  find_package(ignition-gui6 REQUIRED)
  set(IROBOT_CREATE_GUI_TARGET ignition-gui6::ignition-gui6)
  set(IROBOT_CREATE_GUI_AMENT_DEPS)
endif()

find_package(Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)

qt5_add_resources(resources_rcc Create3Hmi.qrc)

add_library(
  Create3Hmi
  SHARED
    ${headers_MOC}
    Create3Hmi.cc
    ${resources_rcc}
)
if(NOT gz_gui_vendor_FOUND)
  target_compile_definitions(Create3Hmi PRIVATE IROBOT_CREATE_USE_IGNITION_GUI=1)
endif()
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Qml_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${Qt5QuickControls2_INCLUDE_DIRS}
)
target_link_libraries(Create3Hmi
  PUBLIC
    ${IROBOT_CREATE_GUI_TARGET}
    ${Qt5Core_LIBRARIES}
    ${Qt5Qml_LIBRARIES}
    ${Qt5Quick_LIBRARIES}
    ${Qt5QuickControls2_LIBRARIES}
)
if(IROBOT_CREATE_GUI_AMENT_DEPS)
  ament_target_dependencies(Create3Hmi ${IROBOT_CREATE_GUI_AMENT_DEPS})
endif()

install(
  TARGETS Create3Hmi
  DESTINATION share/${PROJECT_NAME}/lib
)
