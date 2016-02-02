cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_examples)

find_package(catkin REQUIRED COMPONENTS trac_ik_lib eigen_conversions)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    trac_ik_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${eigen_INCLUDE_DIRS}
)

add_executable(ik_tests src/ik_tests.cpp)
target_link_libraries(ik_tests ${catkin_LIBRARIES})

install(TARGETS ik_tests
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
