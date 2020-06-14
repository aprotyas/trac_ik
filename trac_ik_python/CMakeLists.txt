cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_python)

add_compile_options(-std=c++11 -Wno-deprecated)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  trac_ik_lib
  tf_conversions
)

find_package(SWIG REQUIRED)

if(DEFINED ENV{ROS_PYTHON_VERSION} AND ENV{ROS_PYTHON_VERSION} EQUAL 3)
  # Python 3
  find_package(Python3 COMPONENTS Development)
else()
  # Python 2
  find_package(PythonLibs REQUIRED)
endif()





catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

#set(CMAKE_VERBOSE_MAKEFILE ON)

# SWIG stuff
include(${SWIG_USE_FILE})
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/swig)

# To add trac_ik_lib headers (from this ros answers: http://answers.ros.org/question/201977/include-header-file-from-another-package-indigo/)
include_directories(${catkin_INCLUDE_DIRS})


# Python trac_ik wrapper using SWIG
# Based on: https://github.com/kdhansen/Aseta/blob/36a7a104f3430ff8b54416a07062234673a73762/src/gatsp/CMakeLists.txt
SET_SOURCE_FILES_PROPERTIES(
  swig/trak_ik_wrap.i PROPERTIES CPLUSPLUS ON 
)

if(DEFINED ENV{ROS_PYTHON_VERSION} AND ENV{ROS_PYTHON_VERSION} EQUAL 3)
  include_directories(
    ${Python3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
else()
  include_directories(
    ${PYTHON_INCLUDE_DIRS} # This solved Python.h: No such file or directory
    ${Boost_INCLUDE_DIRS}
  )
endif()



# To overcome (took me hours to notice):
# /opt/ros/indigo/include/kdl/utilities/utility.h:27:19: fatal error: cstdlib: No such file or directory
# ...build/trac_ik_lib/swig/trac_ik_wrapPYTHON_wrap.c:2977:21: fatal error: stdexcept: No such file or directory
set(CMAKE_C_COMPILER ${CMAKE_CXX_COMPILER})

# To force SWIG on generating a C++ file-wrapper so we overcome
# error: expected primary-expression before ‘=’ token
#       TRAC_IK_ns = *((namespace *)(argp));
set(CMAKE_SWIG_FLAGS "-c++")

# This actually makes the compilation of the generated trac_ik_wrapPYTHON_wrap.c happen
if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
  # SWIG_ADD_MODULE is deprecated from 3.8.0
  SWIG_ADD_MODULE(trac_ik_wrap python
  swig/trac_ik_wrap.i
  )
else()
  SWIG_ADD_LIBRARY(trac_ik_wrap
    LANGUAGE python
    SOURCES swig/trac_ik_wrap.i
  )
endif()

# Link the wrapper with the actual library

if(DEFINED ENV{ROS_PYTHON_VERSION} AND ENV{ROS_PYTHON_VERSION} EQUAL 3)
  SWIG_LINK_LIBRARIES(trac_ik_wrap ${Python3_LIBRARIES} ${catkin_LIBRARIES})
else()
  SWIG_LINK_LIBRARIES(trac_ik_wrap ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})
endif()

# The name '_trac_ik_wrap' is given by SWIG, haven't found how to change it
set_target_properties(_trac_ik_wrap
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)


#############
## Install ##
#############

add_custom_command(TARGET _trac_ik_wrap POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/trac_ik_wrap.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

add_custom_command(TARGET _trac_ik_wrap POST_BUILD
                   COMMAND touch ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py
)

# Install the trac_ik_wrap.py generated file
# Needs to be done after creating the python wrapper (usually this is done at the top)


install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/trac_ik_wrap.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/_trac_ik_wrap.so
src/trac_ik_python/trac_ik.py
src/trac_ik_python/__init__.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/test_pkg.py scripts/test_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Testing ##
#############

# TODO

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trac_ik_python.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
