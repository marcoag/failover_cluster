cmake_minimum_required(VERSION 3.5)
project(failover_cluster)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sw_watchdog_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

include_directories(
  include
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

# find_package(<dependency> REQUIRED)

add_library(talker_component SHARED
  src/talker_component.cpp)
target_compile_definitions(talker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(talker_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(talker_component "composition::Talker")
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")

add_library(heartbeat_component SHARED
  src/simple_heartbeat.cpp)
target_compile_definitions(heartbeat_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(heartbeat_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sw_watchdog_msgs")
rclcpp_components_register_nodes(heartbeat_component "simple_heartbeat::SimpleHeartbeat")
set(node_plugins "${node_plugins}simple_heartbeat::SimpleHeartbeat;$<TARGET_FILE:heartbeat_component>\n")

add_library(watchdog_component SHARED
  src/simple_watchdog.cpp)
target_compile_definitions(watchdog_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(watchdog_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sw_watchdog_msgs"
  "rclcpp_lifecycle")
rclcpp_components_register_nodes(heartbeat_component "simple_watchdog::SimpleWatchdog")
set(node_plugins "${node_plugins}simple_watchdog::SimpleWatchdog;$<TARGET_FILE:watchdog_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(manual_composition
  src/manual_composition.cpp)
target_link_libraries(manual_composition
  talker_component
  heartbeat_component)
ament_target_dependencies(manual_composition
  "rclcpp")

add_executable(linktime_composition
  src/linktime_composition.cpp)

set(libs
  talker_component
  heartbeat_component
  watchdog_component)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(libs
    "-Wl,--no-as-needed"
    ${libs}
    "-Wl,--as-needed")
endif()

target_link_libraries(linktime_composition ${libs})
ament_target_dependencies(linktime_composition
  "class_loader"
  "rclcpp"
  "rclcpp_components")

install(TARGETS
  talker_component
  heartbeat_component
  watchdog_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  manual_composition
  linktime_composition
  DESTINATION lib/${PROJECT_NAME})

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
