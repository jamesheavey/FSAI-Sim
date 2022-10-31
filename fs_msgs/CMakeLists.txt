cmake_minimum_required(VERSION 3.5)
project(fs_msgs)

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
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Car.msg"
  "msg/DriverMetadata.msg"
  "msg/Map.msg"
  "msg/GazeboStatus.msg"
  "msg/Driver.msg"
  "msg/Lap.msg"
  "msg/MultiplePoints.msg"
  "msg/SingleCone.msg"
  "msg/MultipleCones.msg"
  "msg/AllCones.msg"
  "msg/VehiclePoseOnMap.msg"
  "msg/MetaKeyValuePair.msg"
  "msg/MetaData.msg"
  "msg/MetaPoint.msg"
  "msg/MetaPointCollection.msg"
  "msg/Report.msg"
  "msg/Penalty.msg"
  "srv/GetCars.srv"
  "srv/GetMaps.srv"
  "srv/GetMap.srv"
  "srv/GetFuzzedMap.srv"
  "srv/LaunchGazebo.srv"
  "srv/StopGazebo.srv"
  "srv/SpawnCar.srv"
  "srv/GetDrivers.srv"
  "srv/StartDriver.srv"
  "srv/StopDriver.srv"
  "srv/RestartDriver.srv"
  "srv/GetReport.srv"
  "srv/AddReport.srv"
  "srv/DeleteReport.srv"
  "srv/GetReports.srv"
  "srv/GetFuzzingConfiguration.srv"
  "srv/SetFuzzingConfiguration.srv"
  "srv/GetSemiStaticMap.srv"

  DEPENDENCIES geometry_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
