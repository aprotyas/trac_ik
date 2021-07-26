cmake_minimum_required(VERSION 3.12)
project(trac_ik_lib)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    kdl_parser
    rclcpp
    urdf
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)
pkg_check_modules(Eigen REQUIRED eigen3)

catkin_package(
  CATKIN_DEPENDS
    kdl_parser
    rclcpp
    urdf
  DEPENDS
    pkg_nlopt
  INCLUDE_DIRS
    include
  LIBRARIES
    trac_ik
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
)

add_library(trac_ik
  src/kdl_tl.cpp
  src/nlopt_ik.cpp
  src/trac_ik.cpp)
target_link_libraries(trac_ik
  ${catkin_LIBRARIES}
  ${pkg_nlopt_LIBRARIES})

# C++20 standard guarantees `std::chrono::system_clock` measures Unix Time
# i.e. `std::chrono::{duration, time_point}` can then replace `boost::posix_time::{time_duration, ptime}`
set_target_properties(trac_ik PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS trac_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
