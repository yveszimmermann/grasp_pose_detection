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


namespace grasp_pose_detection {
/*!
 * detects grasp poses on a object model using a gripper.
 */

class GraspPoseDetection
{

 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GraspPoseDetection(std::vector<std::string>& models_to_detect, std::vector<std::string>& models_detected, std::vector<int>& number_of_grasp_poses);

  /*!
   * Destructor.
   */
  virtual ~GraspPoseDetection();

  /*!
   * detect objects in scene
   */
  bool detectGraspPose();


};

} /* namespace */
