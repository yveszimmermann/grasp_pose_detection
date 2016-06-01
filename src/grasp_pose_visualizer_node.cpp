/*
 * grasp_pose_visualizer_node.cpp
 *
 *  Created on: Jun 1, 2016
 *      Author: yves
 */

#include <grasp_pose_detection/grasp_pose_visualizer.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_pose_visualizer");
  ros::NodeHandle nodeHandle("~");

  grasp_pose_visualizer::GraspPoseVisualizer GraspPoseVisualizer(nodeHandle);

  ros::waitForShutdown();
  return 0;
}






