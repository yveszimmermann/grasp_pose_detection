/*
 * grasp_pose_detection_client_node.cpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#include "grasp_pose_detection/DetectGraspPose.h"
#include <ros/ros.h>
#include <vector>

using namespace grasp_pose_detection;

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

 if (client.call(srv))
  {
   ROS_INFO("Object detection executed.");
   for (int i = 0; i < srv.response.models_detected .size(); i++){
    std::cout << "Model " << srv.response.models_detected[i] << " was detected in the scene." << std::endl;
   }
  }
  else
  {
    ROS_ERROR("Failed to call service grasp_pose_detection");
    return 1;
  }

  return 0;
}


