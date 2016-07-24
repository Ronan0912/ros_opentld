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

# externalproject_add(opentld
#   PREFIX libopentld
#   SVN_REPOSITORY  https://github.com/Ronan0912/OpenTLD/trunk/
#   SVN_REVISION    -rHEAD
#   PATCH_COMMAND   patch -p0 -i ${PROJECT_SOURCE_DIR}/opentld/patches/diff1.patch
#                   &&
#                   patch -p0 -i ${PROJECT_SOURCE_DIR}/opentld/patches/diff2.patch
#   BUILD_IN_SOURCE 1
#   BUILD_COMMAND   cmake --build .
#   INSTALL_COMMAND ${PROJECT_SOURCE_DIR}/opentld/install.sh
#                   ${OPENTLD_OUTPUT_DIR}
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES libopentld
  CATKIN_DEPENDS roscpp cv_bridge std_msgs sensor_msgs tld_msgs
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

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OPENTLD_OUTPUT_DIR}/include/libopentld
)

link_directories(${OPENTLD_OUTPUT_DIR}/lib/libopentld/)

## Declare a cpp executable
add_executable(tld_tracker_node	src/tld_tracker_node.cpp
                                src/main.cpp
)

add_executable(tld_gui_node     src/tld_gui_node.cpp
                                src/base_frame.cpp
                                src/base_frame_graphics_view.cpp
                                ${TLD_QT_HEADERS_MOC}
)

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
