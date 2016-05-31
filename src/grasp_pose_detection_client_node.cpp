/*
 * grasp_pose_detection_client_node.cpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#include "grasp_pose_detection/DetectGraspPose.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
using namespace grasp_pose_detection;

struct grasp_pose
{
  geometry_msgs::Pose grasp_pose;
  std::vector<double> finger_position;  // y position of fingerplate
  double grasp_pose_quality;
};

void readFile(std::string path, std::vector<grasp_pose>& grasp_poses)
{
  std::ifstream in(path, std::ios::in | std::ios::binary);
  typename std::vector<grasp_pose>::size_type size1 = 0;
  in.read((char*) &size1, sizeof(size1));
  grasp_poses.resize(size1);

  for (int i = 0; i < size1; i++) {
    typename std::vector<double>::size_type size2 = 0;
    in.read((char*) &size2, sizeof(size2));
    grasp_poses[i].finger_position.resize(size2);
    in.read((char*) &grasp_poses[i].finger_position[0], grasp_poses[i].finger_position.size() * sizeof(double));
    in.read((char*) &grasp_poses[i].grasp_pose, sizeof(geometry_msgs::Pose));
    in.read((char*) &grasp_poses[i].grasp_pose_quality, sizeof(double));
  }
  in.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_pose_detection_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<DetectGraspPose>("/grasp_pose_detection");
  DetectGraspPose srv;

  std::vector<std::string> model_ids;
  model_ids.push_back("stone_1");
  model_ids.push_back("stone_2");
  model_ids.push_back("stone_3");
  model_ids.push_back("stone_4");
  model_ids.push_back("stone_5");
  model_ids.push_back("stone_6");

  for (std::string id : model_ids) {
    std_msgs::String object;
    object.data = id;
    srv.request.models_to_detect.push_back(object);
  }

  std::cout << "Calling grasp_pose_detection service." << std::endl;

  if (client.call(srv)) {
    ROS_INFO("Grasp pose detection executed.");
    for (int i = 0; i < srv.response.models_detected.size(); i++) {
      std::cout << "Model " << srv.response.models_detected[i] << " was detected in the scene."
                << std::endl;
    }

    std::vector<grasp_pose> grasp_poses;
    std::string path = ros::package::getPath("grasp_pose_detection");
    path = path + "/grasp_poses/stone_6_grasp_poses.gp";
    readFile(path, grasp_poses);
    std::cout << "grasp_poses_size  = " << grasp_poses.size() << std::endl;
    std::cout << "grasp_poses_ori0_x = " << grasp_poses[4].grasp_pose.orientation.x << std::endl;
    std::cout << "grasp_poses_finger_pos = " << grasp_poses[4].finger_position[0] << std::endl;
    std::cout << "grasp_poses_sample = " << grasp_poses[4].grasp_pose_quality << std::endl;

  } else {
    ROS_ERROR("Failed to call service grasp_pose_detection");
    return 1;
  }
  std::cout << "end of main" << std::endl;
  return 0;
}

