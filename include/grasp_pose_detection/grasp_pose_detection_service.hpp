/*
 * grasp_pose_detection.hpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#ifndef INCLUDE_GRASP_POSE_DETECTION_GRASP_POSE_DETECTION_SERVICE_HPP_
#define INCLUDE_GRASP_POSE_DETECTION_GRASP_POSE_DETECTION_SERVICE_HPP_

#include <vector>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include "grasp_pose_detection/GraspPoseDetection.hpp"
#include "grasp_pose_detection/DetectGraspPose.h"

using namespace grasp_pose_detection;
namespace grasp_pose_detection_srv {
/*!
 * Reads PointClouds from topic /camera/depth/points and preprocesses them with temporal median and average filters.
 */
class GraspPoseDetectionSrv
{

 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GraspPoseDetectionSrv(ros::NodeHandle nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~GraspPoseDetectionSrv();

 private:
  ros::NodeHandle nodeHandle_;

 bool callGraspPoseDetection(DetectGraspPose::Request &models_to_detect, DetectGraspPose::Response &detected_model_poses);
};

}/* end namespace grasp_pose_detection_srv */



#endif /* INCLUDE_GRASP_POSE_DETECTION_GRASP_POSE_DETECTION_SERVICE_HPP_ */
