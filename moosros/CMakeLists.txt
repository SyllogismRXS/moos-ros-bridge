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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

find_package(MOOS REQUIRED)
include_directories(${MOOS_INCLUDE_DIRS})
link_directories(${MOOS_LIBRARY_DIRS})
set(LIBS ${LIBS} ${MOOS_LIBRARIES})

INCLUDE(FindPackageMessage) 

message("----------------------------------------------")
if(MOOS_FOUND)
  message("MOOS FOUND!")
else(MOOS_FOUND)
  message("MOOS NOT FOUND")
endif(MOOS_FOUND)
message("----------------------------------------------")

#common commands for building c++ executables and libraries
rosbuild_add_boost_directories()

#FIND_PACKAGE( Boost 1.53 COMPONENTS thread REQUIRED )
#INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
#
#message("----------------------------------------")
#message(" Boost Libs: ${Boost_LIBRARIES}")

#TARGET_LINK_LIBRARIES( Bridge ${Boost_LIBRARIES} )
#target_link_libraries(Bridge MOOSGen)
#target_link_libraries(Bridge MOOSUtility)
#target_link_libraries(Bridge boost_thread)

rosbuild_add_executable( Bridge 
  src/MsgContainer.h 
  src/MOOSNode.cpp 
  src/Bridge.cpp
  )

target_link_libraries(Bridge MOOS)
#rosbuild_link_boost(Bridge thread thread-mt)
rosbuild_link_boost(Bridge thread)
