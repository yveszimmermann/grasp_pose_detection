/*
 * grasp_pose_visualizer.hpp
 *
 *  Created on: Jun 1, 2016
 *      Author: yves
 */

#pragma once


#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include "grasp_pose_detection/GraspPoseDetection.hpp"

using namespace  grasp_pose_detection;

namespace grasp_pose_visualizer {
/*!
 * Reads PointClouds from topic /camera/depth/points and preprocesses them with temporal median and average filters.
 */
class GraspPoseVisualizer
{

 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GraspPoseVisualizer(ros::NodeHandle nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~GraspPoseVisualizer();

 private:
  ros::NodeHandle nodeHandle_;

  double leaf_size_;
  std::string model_name_;
  std::string model_folder_;
  std::string model_path_;

  std::string grasp_pose_folder_;
  std::string grasp_pose_path_;
  int grasp_pose_index_;

  std::vector<GraspPoseDetection::grasp_pose> grasp_poses_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr gripper_;

  double number_of_columns_;
  double number_of_rows_;

  // is loaded by yaml file
  std::vector<GraspPoseDetection::finger_data> gripper_mask_;


 bool visualizeGraspPose(int grasp_pose_index);
 bool loadGraspPoses();
 bool loadModel();
 bool generateGripperCloud();
};

}/* end namespace grasp_pose_detection_srv */
