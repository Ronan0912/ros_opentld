cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

add_definitions(-DPROJECT_PREFIX='"tld_tracker"')
add_definitions(-DPROJECT_VERSION='"0.1"')

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_definitions(-fPIC)

#-------------------------------------------------------------------------------
#find packages
find_package(OpenCV REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui)

#Qt4 specific for the gui
set(TLD_QT_HEADERS src/base_frame.hpp src/ui_baseFrame.h src/base_frame_graphics_view.hpp)
QT4_WRAP_CPP(TLD_QT_HEADERS_MOC ${TLD_QT_HEADERS})
set(SRC_FILES src/base_frame.cpp src/base_frame_graphics_view.cpp ${TLD_QT_HEADERS_MOC})
include(${QT_USE_FILE})

#-------------------------------------------------------------------------------
#Compile and link

#Target 1 : ros_tld_tracker
rosbuild_add_library(main src/main.cpp src/main.hpp)
target_link_libraries(main libopentld cvblobs ${OpenCV_LIBS} cv_bridge)

rosbuild_add_executable(ros_tld_tracker_node src/tld_node.cpp)
target_link_libraries(ros_tld_tracker_node main)

#Target 2 : ros_tld_gui
rosbuild_add_library(gui_lib ${SRC_FILES})

rosbuild_add_executable(ros_tld_gui_node src/tld_gui_node.cpp)
target_link_libraries(ros_tld_gui_node gui_lib ${QT_LIBRARIES})
