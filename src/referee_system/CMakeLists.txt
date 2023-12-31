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

add_executable(mytest 
  src/test.cpp 
  src/MappingTables.cpp 
  src/DataType.cpp
  )

ament_target_dependencies(
  mytest
  )
target_include_directories(mytest
  PRIVATE
    include
    ${PROJECT_SOURCE_DIR}/include/referee_system/
)
install(TARGETS 
mytest
DESTINATION lib/${PROJECT_NAME})

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
