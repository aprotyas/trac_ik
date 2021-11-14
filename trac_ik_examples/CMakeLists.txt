cmake_minimum_required(VERSION 3.12)
project(trac_ik_examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(NLopt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)

add_executable(ik_tests src/ik_tests.cpp)

# C++20 standard guarantees `std::chrono::system_clock` measures Unix Time
# i.e. `std::chrono::{duration, time_point}` can then replace
# `boost::posix_time::{time_duration, ptime}`
set_target_properties(ik_tests PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

target_include_directories(ik_tests PUBLIC
  ${NLOPT_INCLUDE_DIRS}
)

# Needed since NLopt is not an ament package
target_link_libraries(ik_tests
  ${NLOPT_LIBRARIES}
)

ament_target_dependencies(ik_tests
  ament_cmake
  rclcpp
  orocos_kdl
  trac_ik_lib
)

install(TARGETS ik_tests
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
