cmake_minimum_required(VERSION 2.8.3)
project(ximea_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#use PkgConfig
find_package(PkgConfig REQUIRED)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

# This driver uses ximea xiapi, if xiapi was installed
# using https://github.com/fishpepper/ximea_api_installer
# we can use the pkg config file for this:
pkg_search_module(XIMEA xiapi>=4.3.6)
IF (XIMEA_FOUND)
  MESSAGE("-- using ximea xiAPI version ${XIMEA_VERSION} from ${XIMEA_PREFIX}")
  SET(XIMEA_INCLUDE_PATH ${XIMEA_INCLUDE_DIRS})
  SET(XIMEA_LIBRARY_PATH ${XIMEA_LIBRARY_DIRS})
  SET(XIMEA_LIBRARY      ${XIMEA_LIBRARIES})
ELSE (XIMEA_FOUND)
  message(FATAL_ERROR "INFO: xiapi pkg config not found, will use system paths")
  set(XIMEA_INCLUDE_PATH /usr/include)
  set(XIMEA_LIBRARY_PATH /usr/lib)
  set(XIMEA_LIBRARY m3api)
ENDIF (XIMEA_FOUND)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  camera_info_manager
  roslint
  nodelet
  dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system thread)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/xiAPI.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ximea_camera
  CATKIN_DEPENDS roscpp nodelet
  DEPENDS system_lib TinyXML
  DEPENDS boost
)

include_directories(include)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${XIMEA_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## link against lib path
link_directories(${XIMEA_LIBRARY_PATH})

## main node executable
add_executable(ximea_camera_node  src/ros_node.cpp src/driver.cpp src/ros_driver.cpp src/ros_cluster.cpp )

## nodelet library
add_library(ximea_camera_nodelet src/ros_nodelet.cpp src/driver.cpp src/ros_driver.cpp )

## helper to create cfg by dumping camera min/max values
add_executable(ximea_camera_generate_cfg src/generate_cfg.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ximea2_node ximea2_generate_messages_cpp)

# make sure configure headers are built before any node using them
add_dependencies(ximea_camera_node ${PROJECT_NAME}_gencfg)
add_dependencies(ximea_camera_nodelet ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(ximea_camera_node
  ${XIMEA_LIBRARY}
  ${catkin_LIBRARIES}
  yaml-cpp
  ${Boost_LIBRARIES}
)

target_link_libraries(ximea_camera_nodelet
  ${XIMEA_LIBRARY}
  ${catkin_LIBRARIES}
  yaml-cpp
  ${Boost_LIBRARIES}
)

target_link_libraries(ximea_camera_generate_cfg
  ${XIMEA_LIBRARY}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## style guide ##
#############
set(ENABLE_CPPLINT 1)
include(${CMAKE_CURRENT_SOURCE_DIR}/stylecheck/CpplintWrapper.cmake)
CPPLINT_RECURSIVE(cpplint_include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include)
CPPLINT_RECURSIVE(cpplint_src
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}/src)
add_dependencies(ximea_camera_node cpplint_src cpplint_include)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ximea_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS ximea_camera_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY calibrations
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

install(FILES nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ximea2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

###########
##roslint##
###########
set(ROSLINT_CPP_OPTS  "--filter=-whitespace/line_length,-readability/todo" )
roslint_cpp()


#workaround for qtcreator ide integration. do not remove!
file(GLOB_RECURSE NODE_DUMMY_TARGETS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h *.cfg *.yaml *.xml *.launch)
add_custom_target(node_dummy_target SOURCES ${NODE_DUMMY_TARGETS})
