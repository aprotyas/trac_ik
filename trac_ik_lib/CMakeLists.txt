cmake_minimum_required(VERSION 3.12)
project(trac_ik_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(NLopt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

add_library(trac_ik
  src/kdl_tl.cpp
  src/nlopt_ik.cpp
  src/trac_ik.cpp)

# C++20 standard guarantees `std::chrono::system_clock` measures Unix Time
# i.e. `std::chrono::{duration, time_point}` can then replace `boost::posix_time::{time_duration, ptime}`
set_target_properties(trac_ik PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

target_include_directories(trac_ik PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${Eigen3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

target_compile_definitions(trac_ik PRIVATE "TRAC_IK_BUILDING_LIBRARY")

# Needed since NLopt is not an ament package
target_link_libraries(trac_ik
    ${NLOPT_LIBRARIES}
)

ament_target_dependencies(trac_ik
  ament_cmake
  Eigen3
  kdl_parser
  rclcpp
  urdf
)

ament_export_targets(export_trac_ik HAS_LIBRARY_TARGET)

ament_export_libraries(trac_ik)
ament_export_include_directories(include)
ament_export_dependencies(
    ament_cmake
    Eigen3
    eigen3_cmake_module
    kdl_parser
    NLopt
    rclcpp
    urdf
)

install(DIRECTORY include/
    DESTINATION include
)

install(TARGETS trac_ik
    EXPORT export_trac_ik
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
    RUNTIME DESTINATION bin)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
