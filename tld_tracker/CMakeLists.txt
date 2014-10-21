cmake_minimum_required(VERSION 2.8.3)
project(tld_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  tld_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
)

set(OPENTLD_OUTPUT_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../)

include(ExternalProject)

externalproject_add(opentld
  PREFIX libopentld
  SVN_REPOSITORY  https://github.com/Ronan0912/OpenTLD/trunk/
  SVN_REVISION    -rHEAD
  PATCH_COMMAND   patch -p0 -i ${PROJECT_SOURCE_DIR}/opentld/patches/diff1.patch
                  &&
                  patch -p0 -i ${PROJECT_SOURCE_DIR}/opentld/patches/diff2.patch
  BUILD_IN_SOURCE 1
  BUILD_COMMAND   cmake --build .
  INSTALL_COMMAND ${PROJECT_SOURCE_DIR}/opentld/install.sh
                  ${OPENTLD_OUTPUT_DIR}
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
  LIBRARIES libopentld
  CATKIN_DEPENDS roscpp cv_bridge std_msgs sensor_msgs tld_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#Qt4 specific for the GUI
set(TLD_QT_HEADERS  src/base_frame.hpp 
                    src/ui_baseFrame.h
                    src/base_frame_graphics_view.hpp)

QT4_WRAP_CPP(TLD_QT_HEADERS_MOC ${TLD_QT_HEADERS})
include(${QT_USE_FILE})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OPENTLD_OUTPUT_DIR}/include/libopentld
)

link_directories(${OPENTLD_OUTPUT_DIR}/lib/libopentld/)

## Declare a cpp library
# add_library(tld_tracker
#   src/${PROJECT_NAME}/tld_tracker.cpp
# )

## Declare a cpp executable
add_executable(tld_tracker_node	src/tld_tracker_node.cpp
                                src/main.cpp                              
)

add_executable(tld_gui_node     src/tld_gui_node.cpp
                                src/base_frame.cpp 
                                src/base_frame_graphics_view.cpp
                                ${TLD_QT_HEADERS_MOC}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_dependencies(tld_tracker_node opentld)
add_dependencies(tld_tracker_node tld_msgs_generate_messages_cpp)

add_dependencies(tld_gui_node tld_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tld_tracker_node
  libopentld
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

target_link_libraries(tld_gui_node
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)


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
# install(TARGETS tld_tracker tld_tracker_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tld_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
