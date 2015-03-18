cmake_minimum_required(VERSION 2.8.3)
project(laser_odometry)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp sensor_msgs tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

option(STRICT_CENSI_COVARIANCE "Use strict censi covariances" OFF)

#find_package(Eigen REQUIRED)
#add_definitions (${EIGEN_DEFINITIONS})
#include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(src/scanmatching/ANN/include)

# Look for GSL library

#
# this module look for gsl (http://www.gnu.org/software/gsl) support
# it will define the following values
#
# GSL_INCLUDE_DIR = where gsl/gsl_version.h can be found
# GSL_LIBRARY     = the library to link against libgsl
# FOUND_GSL       = set to 1 if gsl is found
#

IF(EXISTS ${PROJECT_CMAKE}/GslConfig.cmake)
  INCLUDE(${PROJECT_CMAKE}/GslConfig.cmake)
ENDIF(EXISTS ${PROJECT_CMAKE}/GslConfig.cmake)

IF(Gsl_INCLUDE_DIRS)

  FIND_PATH(GSL_INCLUDE_DIR gsl/gsl_version.h ${Gsl_INCLUDE_DIRS})
  FIND_LIBRARY(GSL_LIBRARY gsl ${Gsl_LIBRARY_DIRS})

ELSE(Gsl_INCLUDE_DIRS)

  FIND_LIBRARY(GSL_LIBRARY gsl $ENV{GSL_HOME}/lib ${QMC_LIBRARY_PATHS})
  FIND_LIBRARY(GSLCBLAS_LIBRARY gslcblas  $ENV{GSL_HOME}/lib ${QMC_LIBRARY_PATHS})
  FIND_PATH(GSL_INCLUDE_DIR gsl/gsl_version.h $ENV{GSL_HOME}/include ${QMC_INCLUDE_PATHS} )

ENDIF(Gsl_INCLUDE_DIRS)

IF(GSL_INCLUDE_DIR AND GSL_LIBRARY)
  SET(GSL_FOUND 1 CACHE BOOL "Found gsl library")
  SET(GSL_LIBRARY ${GSL_LIBRARY} ${GSLCBLAS_LIBRARY})
ELSE(GSL_INCLUDE_DIR AND GSL_LIBRARY)
  SET(GSL_FOUND 0 CACHE BOOL "Not fount gsl library")
ENDIF(GSL_INCLUDE_DIR AND GSL_LIBRARY)

MARK_AS_ADVANCED(
  GSL_INCLUDE_DIR 
  GSL_LIBRARY 
  GSL_FOUND
  )


# Look for GSL library - END

if(STRICT_CENSI_COVARIANCE)
	add_definitions(-DSTRICT_CENSI_COVARIANCE)
endif(STRICT_CENSI_COVARIANCE)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES laser_odometry
#  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
#)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(laser_odometry
#   src/${PROJECT_NAME}/laser_odometry.cpp
# )


## Declare a cpp executable
add_executable(laser_odometry_node
	src/Main.cpp
	src/LaserOdometryNode.cpp
	src/Config.cpp
	src/scanmatching/ClassicICP.cpp
	src/scanmatching/ICP.cpp
	src/scanmatching/MetricICP.cpp
	src/scanmatching/NearestNeighboursANN.cpp
	src/scanmatching/NearestNeighboursON2.cpp
	src/scanmatching/PointToLineICP.cpp
	src/scanmatching/Rototranslation2D.cpp
	src/scanmatching/Scan.cpp
	src/scanmatching/ANN/src/ANN.cpp
	src/scanmatching/ANN/src/bd_fix_rad_search.cpp
	src/scanmatching/ANN/src/bd_pr_search.cpp
	src/scanmatching/ANN/src/bd_search.cpp
	src/scanmatching/ANN/src/bd_tree.cpp
	src/scanmatching/ANN/src/brute.cpp
	src/scanmatching/ANN/src/kd_dump.cpp
	src/scanmatching/ANN/src/kd_fix_rad_search.cpp
	src/scanmatching/ANN/src/kd_pr_search.cpp
	src/scanmatching/ANN/src/kd_search.cpp
	src/scanmatching/ANN/src/kd_split.cpp
	src/scanmatching/ANN/src/kd_tree.cpp
	src/scanmatching/ANN/src/kd_util.cpp
	src/scanmatching/ANN/src/perf.cpp
	src/scanmatching/gpc/gpc.c
	src/scanmatching/gpc/gpc_utils.c)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(laser_odometry_node laser_odometry_generate_messages_cpp)

add_dependencies(laser_odometry_node scanmatching/ANN/ANNx.h)

## Specify libraries to link a library or executable target against
target_link_libraries(laser_odometry_node
   ${catkin_LIBRARIES}
)

target_link_libraries(laser_odometry_node gsl gslcblas)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS laser_odometry laser_odometry_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_laser_odometry.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
