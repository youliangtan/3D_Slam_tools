cmake_minimum_required(VERSION 2.8.3)
project(3D_Slam_tools)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  tf2
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL #OCTOMAP
)


link_directories(${PCL_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OCTOMAP_DEFINITIONS})
add_definitions( -march=native )


add_executable(pcd2octomap_node src/pcd2octomap_node.cpp)
target_link_libraries(pcd2octomap_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

add_executable(pcd2octomap src/pcd2octomap.cpp)
target_link_libraries(pcd2octomap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES}  )

add_executable(pc_frame_conversion_node src/pc_frame_conversion_node.cpp)
target_link_libraries(pc_frame_conversion_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

add_executable(pcd2ply src/pcd2ply.cpp)
target_link_libraries(pcd2ply ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(ply2pcd src/ply2pcd.cpp)
target_link_libraries(ply2pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(imu_TFpublisher src/imu_TFpublisher.cpp)
target_link_libraries(imu_TFpublisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(carto_map_saver src/carto_map_saver.cpp)
target_link_libraries(carto_map_saver ${catkin_LIBRARIES} ${PCL_LIBRARIES})
