cmake_minimum_required(VERSION 2.8.3)
project(grasp_pose_detection)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(PCL REQUIRED)

add_service_files(
  FILES
  DetectGraspPose.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  	include
  LIBRARIES
  	grasp_pose_detection
  CATKIN_DEPENDS
  	pcl_conversions
  	pcl_ros 
  	roscpp
  	rospy 
  	std_msgs
  	message_runtime
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  			src/grasp_pose_detection_service_node.cpp
			src/grasp_pose_detection_service.cpp
		    src/GraspPoseDetection.cpp
)
add_dependencies(${PROJECT_NAME} grasp_pose_detection_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# Detection Service
	add_executable(
		grasp_pose_detection_service
			src/grasp_pose_detection_service_node.cpp
			src/grasp_pose_detection_service.cpp
		    src/GraspPoseDetection.cpp
	)

	add_dependencies(grasp_pose_detection_service grasp_pose_detection_gencpp)
	target_link_libraries(grasp_pose_detection_service ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Detection Client
	add_executable(
		grasp_pose_detection_client
			src/grasp_pose_detection_client_node.cpp
	)

	add_dependencies(grasp_pose_detection_client grasp_pose_detection_gencpp)
	target_link_libraries(grasp_pose_detection_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})
	
	
# Visualizer
	add_executable(
		grasp_pose_visualizer
			src/grasp_pose_visualizer_node.cpp
			src/grasp_pose_visualizer.cpp
		    src/GraspPoseDetection.cpp
	)

	add_dependencies(grasp_pose_visualizer grasp_pose_visualizer_gencpp)
	target_link_libraries(grasp_pose_visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
