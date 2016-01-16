cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_examples)

find_package(catkin REQUIRED COMPONENTS trac_ik_lib eigen_conversions)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${eigen_INCLUDE_DIRS}
)

add_executable(ik_tests src/ik_tests.cpp)
target_link_libraries(ik_tests ${catkin_LIBRARIES})