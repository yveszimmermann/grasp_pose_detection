/*
 * grasp_pose_detection_node.cpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#include <grasp_pose_detection/grasp_pose_detection_service.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_pose_detection_service");
  ros::NodeHandle nodeHandle("~");

  grasp_pose_detection_srv::GraspPoseDetectionSrv GraspPoseDetectionSrv(nodeHandle);

  ros::waitForShutdown();
  return 0;
}



