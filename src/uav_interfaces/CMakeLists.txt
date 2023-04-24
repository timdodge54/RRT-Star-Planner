cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoPilotCommands.msg"
  "msg/BuildingWorld.msg"
  "msg/Compass.msg"
  "msg/ControlSurfaceCommands.msg"
  "msg/Gps.msg"
  "msg/Magnetometer.msg"
  "msg/PathSegment.msg"
  "msg/Pressure.msg"
  "msg/UavState.msg"
  "msg/UavStateWrench.msg"
  "msg/UavWaypoint.msg"
  "msg/UavWaypoints.msg"
  "msg/WindVector.msg"
  "msg/LineOfSight.msg"
  "msg/LineOfSightArray.msg"
  "srv/SetUavState.srv"
  "msg/PoseWithCovarianceStampedArray.msg"
  "msg/BuoyPoseRadiusArray.msg"
  "srv/SetClockParams.srv"
  "srv/ToggleEllipse.srv"
  "srv/LinCovMonteCarloCall.srv"
  "srv/SelectAnalysisPlot.srv"

  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  unique_identifier_msgs
  std_msgs
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

ament_package()
