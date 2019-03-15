cmake_minimum_required(VERSION 2.8.3)
project(laser-odom_calibration)

# Detection of CSM library using CMake routines
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(CSM REQUIRED csm)

##IF(${CSM_FOUND})
#        MESSAGE("CSM_LIBRARY_DIRS: ${CSM_LIBRARY_DIRS}")
#        MESSAGE("CSM_LIBRARIES: ${CSM_LIBRARIES}")
#        MESSAGE("CSM_INCLUDE_DIRS: ${CSM_INCLUDE_DIRS}")

#        INCLUDE_DIRECTORIES(${CSM_INCLUDE_DIRS})
#        LINK_DIRECTORIES(${CSM_LIBRARY_DIRS})

##ELSE(${CSM_FOUND})
#        MESSAGE(SEND_ERROR "CSM not found. Make sure PKG_CONFIG_PATH contains the directory where the file 'csm.pc' can be found.")
##ENDIF(${CSM_FOUND})

SET(CSM_SRC 
  src/csm/laser_data.cpp
  src/csm/math_utils.cpp
  src/csm/math_utils_gsl.cpp
  src/csm/utils.cpp
  src/csm/logging.cpp
  src/csm/sm_options.cpp
  src/csm/orientation.cpp
  src/csm/clustering.cpp)

SET(EGSL_SRC
	src/egsl/egsl.cpp
	src/egsl/egsl_conversions.cpp
	src/egsl/egsl_misc.cpp
	src/egsl/egsl_ops.cpp)

SET(GPC_SRC
	src/gpc/gpc.cpp
	src/gpc/gpc_utils.cpp)

SET(ICP_SRC
  src/icp/icp.cpp
  src/icp/icp_corr_dumb.cpp
  src/icp/icp_corr_tricks.cpp
  src/icp/icp_covariance.cpp
  src/icp/icp_debug.cpp
  src/icp/icp_loop.cpp
  src/icp/icp_outliers.cpp)

SET(OPTIONS_SRC  
   src/options/options.cpp
   src/options/options_interface.cpp)

ADD_LIBRARY(csm_eigen
	${CSM_SRC}
	${EGSL_SRC} 
	${GPC_SRC}
	${ICP_SRC}
	${OPTIONS_SRC})

# Various flags

set(CMAKE_CXX_FLAGS "-std=c++11")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O2 ")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    rosbag
    )


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

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

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_odom_calib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# A test
# ADD_EXECUTABLE(test_calib src/main.cpp)
# TARGET_LINK_LIBRARIES(test_calib ${CSM_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(io_test src/io_test.cpp src/io.cpp src/utils.cpp src/scan_match.cpp src/synchronizer.cpp src/solver.cpp)
TARGET_LINK_LIBRARIES(io_test ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR} csm_eigen)
