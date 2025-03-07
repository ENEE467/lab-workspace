cmake_minimum_required(VERSION 3.8)
project(lab_rviz_panels)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set(RVIZ_PANEL_DEPS
  rviz_common
  Qt5Core
  Qt5Gui
  Qt5Widgets
)

set(CORE_DEPENDENCIES
  ament_cmake
  rclcpp
)

set(LAB7_PANEL_DEPS
  "${RVIZ_PANEL_DEPS}"
  lab7
)

# More panel dependencies...

set(UI_FILES
  ui/hand_eye_calib_panel.ui
)

set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/hand_eye_calib_panel.hpp
)

# Find dependencies
foreach(dependency IN ITEMS ${CORE_DEPENDENCIES} ${LAB7_PANEL_DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

qt5_wrap_ui(UI_HEADERS ${UI_FILES})
qt5_wrap_cpp(MOCS ${${PROJECT_NAME}_HEADERS})

# Hand-eye calibration panel
add_library(hand_eye_calib_panel SHARED
  src/hand_eye_calib_panel.cpp
  ${UI_HEADERS}
  ${MOCS}
)

ament_target_dependencies(hand_eye_calib_panel
  "${CORE_DEPENDENCIES}"
  "${LAB7_PANEL_DEPS}"
)

target_link_libraries(hand_eye_calib_panel rviz_common::rviz_common)
target_compile_definitions(hand_eye_calib_panel PRIVATE QT_NO_KEYWORDS)

target_include_directories(hand_eye_calib_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# More panels...

pluginlib_export_plugin_description_file(rviz_common plugin.xml)
install(TARGETS hand_eye_calib_panel)

ament_package()
