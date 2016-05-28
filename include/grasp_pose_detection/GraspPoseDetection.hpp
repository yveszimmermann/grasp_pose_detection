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
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
  };

  struct grasp_pose
  {
    geometry_msgs::Pose grasp_pose;
    std::vector<double> finger_position;
  };

  struct finger_data{
    Eigen::Vector3f finger_initial_position;
    Eigen::Vector3f finger_plate_normal;
    float trajectory_angle; //angle between normal and trajectory, used for rotation of model around the x-axis
    float finger_height;
    float finger_width;
  };

    std::vector<finger_data> gripper_mask;


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

  /*!
   * detect objects in scene
   */
  bool loadModelData();

  /*!
    * detect objects in scene
    */
  bool DownSample();

  /*!
   * detect objects in scene
   */
  bool computeNormals();

  /*!
   * detect objects in scene
   */
  bool setModelPath(std::string model_path);




 private:

  std::vector<object_data> models_;
  std::vector<grasp_pose> grasp_pose_;
  std::vector<finger_data> finger_data_;
  std::vector<Eigen::Vector3f> current_grasp_direction_;
  std::string model_path_;
  std::vector<std::string> models_to_detect_;


  // Parameters
  double normal_search_radius_;
  double normal_angle_threshold_;
  double leaf_size_;

};

} /* namespace */
