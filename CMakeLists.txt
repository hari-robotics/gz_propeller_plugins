cmake_minimum_required(VERSION 3.8)
project(gz_propeller_plugins)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

# Use C11 and C++ 17 for building
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
# Generating compile commands for clangd intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

file(GLOB_RECURSE SOURCES "src/*.cc" "src/*.cpp")
add_library(gz_propeller_plugins SHARED ${SOURCES})

ament_target_dependencies(gz_propeller_plugins
  gazebo_dev
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
