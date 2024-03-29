cmake_minimum_required(VERSION 3.8)
project(referee_system_test_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(my_msg_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(referee_system_test_client 
  src/test_client.cpp
)
ament_target_dependencies(
  referee_system_test_client
  ament_cmake
  rclcpp
  my_msg_interface
  )
target_include_directories(referee_system_test_client
  PRIVATE
    include
    ${PROJECT_SOURCE_DIR}/include/referee_system_test_client/
)
install(TARGETS 
referee_system_test_client
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
