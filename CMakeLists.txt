cmake_minimum_required(VERSION 2.8.3)
project(registration_localization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rviz
  std_srvs
  qt_build
  pcl_conversions 
  pcl_ros 
dynamic_reconfigure
)
find_package(PCL 1.7 REQUIRED)

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
generate_dynamic_reconfigure_options(
  cfg/test.cfg
  #...
)
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
#  INCLUDE_DIRS include
#  LIBRARIES radbot_dashboard
   CATKIN_DEPENDS std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  include/${PROJECT_NAME}
)
include_directories(include ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
#add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/dashboard_panel.h
  include/${PROJECT_NAME}/point_selector.h
  include/${PROJECT_NAME}/save_marker.hpp
  include/${PROJECT_NAME}/alignment_node.hpp
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/dashboard_panel.cpp
  src/point_selector.cpp
  src/save_marker.cpp
  src/alignment_node.cpp
  ${MOC_FILES}
)

set(${PROJECT_NAME}_UIS
  src/dashboard_panel.ui
)

qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})


add_executable(saveapcd src/savepcd.cpp)
add_executable(testing src/testing2.cpp)
add_executable(algorithm src/algorithm.cpp)
add_executable(matching_test src/matching.cpp)
add_executable(planes src/planes.cpp)
add_executable(test_planes src/planes2.cpp)
add_executable(showclouds src/showclouds.cpp)
add_executable(showcloud src/showcloud.cpp)
add_executable(cloudloop src/cloudloop.cpp)
add_executable(searchloop src/searchloop.cpp)
add_executable(searchall src/searchall.cpp)
add_executable(segmentation src/segmentation.cpp)
add_executable(keypoints src/show_keypoints.cpp)

add_dependencies(searchloop ${PROJECT_NAME}_gencfg)
add_dependencies(searchall ${PROJECT_NAME}_gencfg)
add_dependencies(cloudloop ${PROJECT_NAME}_gencfg)

## Declare a cpp library
 add_library(${PROJECT_NAME} 
   ${SOURCE_FILES}
   ${${PROJECT_NAME}_UIS_H}
 )

#add_dependencies(${PROJECT_NAME} frontier_exploration_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
# add_executable(radbot_dashboard_node src/radbot_dashboard_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(radbot_dashboard_node radbot_dashboard_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES}
 )

target_link_libraries(saveapcd  ${catkin_LIBRARIES})
target_link_libraries(algorithm  ${catkin_LIBRARIES})
target_link_libraries(matching_test  ${catkin_LIBRARIES})
target_link_libraries(showclouds  ${catkin_LIBRARIES})
target_link_libraries(showcloud  ${catkin_LIBRARIES})
target_link_libraries(cloudloop  ${catkin_LIBRARIES})
target_link_libraries(searchloop  ${catkin_LIBRARIES})
target_link_libraries(searchall  ${catkin_LIBRARIES})
target_link_libraries(planes  ${catkin_LIBRARIES})
target_link_libraries(test_planes  ${catkin_LIBRARIES})
target_link_libraries(segmentation  ${catkin_LIBRARIES})
target_link_libraries(testing  ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
target_link_libraries(keypoints  ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so )

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

#install(DIRECTORY icons/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

#install(PROGRAMS scripts/send_test_msgs.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radbot_dashboard.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
