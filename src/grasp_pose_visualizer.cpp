/*
 * grasp_pose_visualizer.cpp
 *
 *  Created on: Jun 1, 2016
 *      Author: yves
 */

#include "grasp_pose_detection/grasp_pose_visualizer.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

using namespace grasp_pose_detection;

namespace grasp_pose_visualizer {

GraspPoseVisualizer::GraspPoseVisualizer(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      model_(new pcl::PointCloud<pcl::PointXYZ>),
      gripper_(new pcl::PointCloud<pcl::PointXYZ>),
      number_of_columns_(40),
      number_of_rows_(40)
{
  // Read Parameters
  XmlRpc::XmlRpcValue gripper_mask;
  nodeHandle_.getParam("/grasp_pose_visualizer/leaf_size", leaf_size_);
  nodeHandle_.getParam("/grasp_pose_visualizer/model_name", model_name_);
  nodeHandle_.getParam("/grasp_pose_visualizer/model_folder", model_folder_);
  nodeHandle_.getParam("/grasp_pose_visualizer/grasp_pose_folder", grasp_pose_folder_);
  nodeHandle_.getParam("/grasp_pose_visualizer/grasp_pose_index", grasp_pose_index_);
  nodeHandle_.getParam("/grasp_pose_visualizer/gripper_mask_1", gripper_mask);

  // Set model path
  std::string path = ros::package::getPath("grasp_pose_detection");
  model_path_ = path + model_folder_;
  model_path_ = model_path_ + model_name_;
  model_path_ = model_path_ + ".pcd";

  grasp_pose_path_ = path + grasp_pose_folder_;
  grasp_pose_path_ = grasp_pose_path_ + model_name_;
  grasp_pose_path_ = grasp_pose_path_ + "_grasp_poses.gp";

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
    finger.max_grasp_angle = static_cast<double>(gripper_mask[i]["max_grasp_angle"]);
    finger.pinch_group = static_cast<int>(gripper_mask[i]["pinch_group"]);
    finger.collision_distance = static_cast<double>(gripper_mask[i]["collision_distance"]);
    gripper_mask_.push_back(finger);
  }

  loadModel();
  loadGraspPoses();
  generateGripperCloud();
  std::cout << "Number of gripper points: " << gripper_->size() << std::endl;
  std::cout << "Quality: " << grasp_poses_[0].grasp_pose_quality << std::endl;
  visualizeGraspPose(grasp_pose_index_);
}

GraspPoseVisualizer::~GraspPoseVisualizer()
{
}

bool GraspPoseVisualizer::visualizeGraspPose(int grasp_pose_index)
{
  pcl::visualization::PCLVisualizer::Ptr visualizer(
      new pcl::visualization::PCLVisualizer("Grasp_Pose"));

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_model(model_, 255, 255, 255);
  visualizer->addPointCloud(model_, color_model, "model");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_gripper(gripper_, 0, 0, 255);
  visualizer->addPointCloud(gripper_, color_gripper, "gripper");

  while (ros::ok()) {
    visualizer->spinOnce();
  }
  return true;
}

bool GraspPoseVisualizer::loadGraspPoses()
{
  std::cout << "Load Grasp poses from: " << grasp_pose_path_ << std::endl;
  std::ifstream in(grasp_pose_path_, std::ios::in | std::ios::binary);
  typename std::vector<GraspPoseDetection::grasp_pose>::size_type size1 = 0;
  in.read((char*) &size1, sizeof(size1));
  grasp_poses_.resize(size1);

  for (int i = 0; i < size1; i++) {
    typename std::vector<double>::size_type size2 = 0;
    in.read((char*) &size2, sizeof(size2));
    grasp_poses_[i].finger_position.resize(size2);
    in.read((char*) &grasp_poses_[i].finger_position[0],
            grasp_poses_[i].finger_position.size() * sizeof(double));
    in.read((char*) &grasp_poses_[i].grasp_pose, sizeof(geometry_msgs::Pose));
    in.read((char*) &grasp_poses_[i].grasp_pose_quality, sizeof(double));
  }
  in.close();
  return true;
}

bool GraspPoseVisualizer::loadModel()
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(model_path_, *model_) == -1) {
    PCL_ERROR("Couldn't read input file base \n");
    std::cout << model_->size() << std::endl;
    return (-1);
  }
  return true;
}

bool GraspPoseVisualizer::generateGripperCloud()
{

  for (int i = 0; i < gripper_mask_.size(); i++) {
    Eigen::Vector3d contact_position;
    double column_distance = gripper_mask_[i].plate_width/(number_of_columns_ - 1);
    double row_distance = gripper_mask_[i].plate_height/(number_of_rows_ - 1);

    contact_position.x() = gripper_mask_[i].initial_position.x();
    contact_position.y() = grasp_poses_[grasp_pose_index_].finger_position[i];
    contact_position.z() = gripper_mask_[i].initial_position.z()
        + sin(gripper_mask_[i].trajectory_angle)
            * fabs(grasp_poses_[grasp_pose_index_].finger_position[i]
                - gripper_mask_[i].initial_position.y());
    for (int j = 0; j < number_of_columns_; j++) {
      for (int k = 0; k < number_of_rows_; k++) {
        pcl::PointXYZ new_point;
        new_point.x  = contact_position.x() - gripper_mask_[i].plate_width/2 + column_distance * j;
        new_point.y  = contact_position.y();
        new_point.z  = contact_position.z() - gripper_mask_[i].plate_height/2 + row_distance * k;
        gripper_->push_back(new_point);
      }
    }
  }
  Eigen::Quaternionf quat;
  quat.x() = grasp_poses_[grasp_pose_index_].grasp_pose.orientation.x;
  quat.y() = grasp_poses_[grasp_pose_index_].grasp_pose.orientation.y;
  quat.z() = grasp_poses_[grasp_pose_index_].grasp_pose.orientation.z;
  quat.w() = grasp_poses_[grasp_pose_index_].grasp_pose.orientation.w;
  Eigen::Matrix3f rot_mat = quat.toRotationMatrix();

  Eigen::Vector3f position;
  position.x() = grasp_poses_[grasp_pose_index_].grasp_pose.position.x;
  position.y() = grasp_poses_[grasp_pose_index_].grasp_pose.position.y;
  position.z() = grasp_poses_[grasp_pose_index_].grasp_pose.position.z;
  Eigen::Vector4f unit;
  unit << 0,0,0,1;
  Eigen::Matrix4f tf_mat;
  tf_mat.block<3,3>(0,0) = rot_mat;
  tf_mat.block<3,1>(0,3) = position;
  tf_mat.block<1,4>(3,0) = unit;

  pcl::transformPointCloud(*gripper_, *gripper_, tf_mat);
  return true;

}

}/*end namespace*/

