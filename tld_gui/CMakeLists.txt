cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

add_definitions(-DPROJECT_PREFIX='"tld_predator"')
add_definitions(-DPROJECT_VERSION='"0.1"')

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#------------------------------------------------------------------------------
#find packages
find_package(Qt4 COMPONENTS QtCore QtGui)

#-------------------------------------------------------------------------------
#Qt4 specific
set(TLD_QT_HEADERS src/base_frame.hpp src/ui_baseFrame.h src/base_frame_graphics_view.hpp)
QT4_WRAP_CPP(TLD_QT_HEADERS_MOC ${TLD_QT_HEADERS})

set(SRC_FILES src/base_frame.cpp src/base_frame_graphics_view.cpp ${TLD_QT_HEADERS_MOC})
set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)

include(${QT_USE_FILE})
include_directories(${INCLUDE_DIRS})

#-------------------------------------------------------------------------------
#common commands for building c++ executables and libraries
rosbuild_add_library(ros_tld_gui_node ${SRC_FILES})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_link_flags(ros_tld_gui_node -DWITH_QT)
rosbuild_add_executable(ros_tld_gui_node src/tld_gui_node.cpp)
target_link_libraries(ros_tld_gui_node ${QT_LIBRARIES})
