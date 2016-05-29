/*
 * grasp_pose_detection_service.cpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include "grasp_pose_detection/GraspPoseDetection.hpp"
#include "grasp_pose_detection/DetectGraspPose.h"
#include "grasp_pose_detection/grasp_pose_detection_service.hpp"

using namespace grasp_pose_detection;

namespace grasp_pose_detection_srv {

GraspPoseDetectionSrv::GraspPoseDetectionSrv(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{
  // Read Parameters
  XmlRpc::XmlRpcValue gripper_mask;
  nodeHandle_.getParam("/grasp_pose_detection_service/leaf_size", leaf_size_);
  nodeHandle_.getParam("/grasp_pose_detection_service/number_of_equator_points", number_of_equator_points_);
  nodeHandle_.getParam("/grasp_pose_detection_service/model_folder", model_folder_);
  nodeHandle_.getParam("/grasp_pose_detection_service/gripper_mask_1", gripper_mask);

  // Set model path
  std::string path = ros::package::getPath("grasp_pose_detection");
  model_path_ = path + model_folder_;

  // encode gripper mask
    for (int i = 0; i < gripper_mask.size(); ++i) {
      GraspPoseDetection::finger_data finger;
      finger.id = static_cast<std::string>(gripper_mask[i]["id"]);
      finger.initial_position.x() = static_cast<double>(gripper_mask[i]["initial_position"]["x"]);
      finger.initial_position.y() = static_cast<double>(gripper_mask[i]["initial_position"]["y"]);
      finger.initial_position.z() = static_cast<double>(gripper_mask[i]["initial_position"]["z"]);
      finger.plate_normal.x() = static_cast<double>(gripper_mask[i]["plate_normal"]["x"]);
      finger.plate_normal.y() = static_cast<double>(gripper_mask[i]["plate_normal"]["y"]);
      finger.plate_normal.z() = static_cast<double>(gripper_mask[i]["plate_normal"]["z"]);
      finger.trajectory_angle = static_cast<double>(gripper_mask[i]["trajectory_angle"]);
      finger.plate_height = static_cast<double>(gripper_mask[i]["plate_height"]);
      finger.plate_width = static_cast<double>(gripper_mask[i]["plate_width"]);
      gripper_mask_.push_back(finger);
    }

  ros::ServiceServer service = nodeHandle_.advertiseService("/grasp_pose_detection", &GraspPoseDetectionSrv::callGraspPoseDetection, this);
  ROS_INFO("Ready to detect grasp poses.");
  ros::spin();
}

GraspPoseDetectionSrv::~GraspPoseDetectionSrv()
{
}

bool GraspPoseDetectionSrv::callGraspPoseDetection(DetectGraspPose::Request &req,
                                                   DetectGraspPose::Response &resp)
{
  ROS_INFO("Service called.");
  std::vector<std::string> models_to_detect_vec;
  std::vector<std::string> models_detected_vec;
  std::vector<int> number_of_grasp_poses_vec;

  for (int i = 0; i  < req.models_to_detect.size(); i++){
    models_to_detect_vec.push_back(req.models_to_detect[i].data);
  }

  GraspPoseDetection GraspPoseDetection(models_to_detect_vec, models_detected_vec, number_of_grasp_poses_vec);
  GraspPoseDetection.setModelPath(model_path_);
  GraspPoseDetection.setGripperMask(gripper_mask_);
  GraspPoseDetection.setNumberOfEquatorPoints(number_of_equator_points_);
  GraspPoseDetection.setLeafSize(leaf_size_);
  GraspPoseDetection.detectGraspPose();

  for (int i = 0; i < models_detected_vec.size(); i++) {
    std::cout << models_detected_vec[i] << " as " << number_of_grasp_poses_vec[i] << std::endl;
  }

 // write results to response
  for (int i = 0; i  < resp.models_detected.size(); i++){
    std_msgs::String model_name;
    std_msgs::Int32 number_of_grasp_poses;

    model_name.data = models_detected_vec[i];
    number_of_grasp_poses.data = number_of_grasp_poses_vec[i];

    resp.models_detected.push_back(model_name);
    resp.number_of_grasp_poses.push_back(number_of_grasp_poses);
  }

  return true;
}

}/*end namespace*/


