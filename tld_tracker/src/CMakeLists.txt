cmake_minimum_required(VERSION 2.6)

#-------------------------------------------------------------------------------
#Compile and link
rosbuild_add_library(main main.cpp main.hpp)
target_link_libraries(main libopentld cvblobs ${OpenCV_LIBS} cv_bridge)

rosbuild_add_executable(ros_tld_tracker_node tld_node.cpp)
target_link_libraries(ros_tld_tracker_node main)

