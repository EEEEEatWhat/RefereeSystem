cmake_minimum_required(VERSION 3.8)
project(referee_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)  
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_msg_interface REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(run_referee_system 
  # src/test.cpp 
  src/referee_system.cpp
  src/MappingTables.cpp 
  src/DataType.cpp
  )

ament_target_dependencies(
  run_referee_system
  ament_cmake
  rclcpp
  std_msgs
  geometry_msgs
  my_msg_interface
  nav2_msgs

  )
target_include_directories(run_referee_system
  PRIVATE
    include
    ${PROJECT_SOURCE_DIR}/include/referee_system/
)
install(TARGETS 
run_referee_system
DESTINATION lib/${PROJECT_NAME})


# # # # # # # # # # # # # # # # # # # # # # # # # #
add_executable(runtest 
  src/test.cpp 
  # src/referee_system.cpp
  src/MappingTables.cpp 
  src/DataType.cpp
  )

ament_target_dependencies(
  runtest
  ament_cmake
  rclcpp
  std_msgs
  geometry_msgs
  my_msg_interface

  )
target_include_directories(runtest
  PRIVATE
    include
    ${PROJECT_SOURCE_DIR}/include/referee_system/
)
install(TARGETS 
runtest
DESTINATION lib/${PROJECT_NAME})
# # # # # # # # # # # # # # # # # # # # # # # # # #
install(DIRECTORY 
launch DESTINATION 
share/${PROJECT_NAME})

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
