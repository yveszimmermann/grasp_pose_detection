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

 public:

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

  struct finger_data
  {
    std::string id;
    Eigen::Vector3d initial_position;
    Eigen::Vector3d plate_normal;
    double trajectory_angle;  //angle between normal and trajectory, used for rotation of model around the x-axis
    double plate_height;
    double plate_width;
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
  bool computeGraspDirections();

  /*!
   * detect objects in scene
   */
  bool setModelPath(std::string model_path);

  /*!
   * detect objects in scene
   */
  bool setGripperMask(std::vector<finger_data> gripper_mask);

 private:

  std::string model_path_;
  std::vector<std::string> models_to_detect_;
  std::vector<finger_data> gripper_mask_;
  std::vector<object_data> models_;

  std::vector<Eigen::Vector3d> grasp_directions_;
  std::vector<grasp_pose> grasp_poses_;


  // Parameters
  double normal_search_radius_;
  double normal_angle_threshold_;
  double leaf_size_;
  double number_of_equator_points_; //ammount of points on the meridian of the geodesic grid

};

} /* namespace */
