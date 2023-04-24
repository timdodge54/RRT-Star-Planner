cmake_minimum_required(VERSION 3.8)
project(suas_rviz_plugins)

# Default to C++14 (maybe remove)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(ament_lint_common REQUIRED)
find_package(ament_lint_auto REQUIRED)

set(rviz_plugins_headers_to_moc
  include/suas_rviz_plugins/waypoint_panel.hpp
  include/suas_rviz_plugins/control_panel.hpp
  include/suas_rviz_plugins/state_panel.hpp
)

include_directories(
  include
)

# Create plugins library
set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/waypoint_panel.cpp
  src/control_panel.cpp
  src/state_panel.cpp
  ${rviz_plugins_headers_to_moc}
)

set(dependencies
  geometry_msgs
  nav_msgs
  pluginlib
  Qt5
  rclcpp
  rclcpp_lifecycle
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  uav_interfaces
)



ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  map_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  uav_interfaces
)

ament_package()
