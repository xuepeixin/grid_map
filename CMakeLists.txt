cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(grid_map)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

# init_qt: Let's do the CMake job for us
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  rospy  
  roscpp
  pcl_ros
  message_generation
)

catkin_package(
  CATKIN_DEPENDS 
)
find_package( OpenCV REQUIRED )
include_directories(include ${OpenCV_INCLUDE_DIRS})


include_directories(${catkin_INCLUDE_DIRS}
                    ${catkin_LIBRARIES})


add_executable(cloud2map  src/cloud2map.cpp
                          src/ground_filter.cpp
                          src/main.cpp
                          src/DBSCAN.cpp
                          src/convex_hull.cpp)

target_link_libraries(cloud2map ${catkin_LIBRARIES})
install(TARGETS cloud2map
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})



add_executable(path_drawer  src/path_drawer.cpp
                          )

target_link_libraries(path_drawer ${catkin_LIBRARIES})
install(TARGETS path_drawer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})