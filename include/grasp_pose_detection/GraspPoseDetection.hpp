/*
 * GraspPoseDetection.hpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#pragma once

// STD
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

namespace grasp_pose_detection {
/*!
 * detects grasp poses on a object model using a gripper.
 */

class GraspPoseDetection
{

  typedef pcl::PointXYZ PointType;

  struct object_data
  {
    pcl::PointCloud<PointType> point_cloud;
    pcl::PointCloud<pcl::Normal> normals;
  };

  struct grasp_pose
  {
    geometry_msgs::Pose grasp_pose;
    std::vector finger_position;
  };

  struct finger_data{
    std::vector finger_initial_position;
    Eigen::Vector3f finger_plate_normal;
    float trajectory_angle; //angle between normal and trajectory, used for rotation of model around the x-axis
    float finger_height;
    float finger_width;
  };

  struct gripper_mask{
    std::vector<finger_data> finger_data;
  };

 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GraspPoseDetection(std::vector<std::string>& models_to_detect,
                     std::vector<std::string>& models_detected,
                     std::vector<int>& number_of_grasp_poses);

  /*!
   * Destructor.
   */
  virtual ~GraspPoseDetection();

  /*!
   * detect objects in scene
   */
  bool detectGraspPose();

 private:

  std::vector<object_data> models_;
  std::vector<grasp_pose> grasp_pose_;



};

} /* namespace */
