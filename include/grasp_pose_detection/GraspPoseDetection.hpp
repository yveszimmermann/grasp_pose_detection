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
    pcl::PointCloud<PointType>::Ptr normals_cloud;
  };

  struct grasp_pose
  {
    geometry_msgs::Pose grasp_pose;
    std::vector<double> finger_position;  // y position of fingerplate
    double grasp_pose_quality;
  };

  struct finger_data
  {
    std::string id;
    Eigen::Vector3d initial_position;
    Eigen::Vector3d plate_normal;
    double trajectory_angle;  //angle between normal and trajectory, used for rotation of model around the x-axis
    double plate_height;
    double plate_width;
    double max_grasp_angle;
    int pinch_group;
  };

 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  GraspPoseDetection(std::vector<std::string>& models_to_detect);

  /*!
   * Destructor.
   */
  virtual ~GraspPoseDetection();

  /*!
   * detect objects in scene
   */
  bool detectGraspPose(std::vector<std::string>& models_detected,
                       std::vector<int>& number_of_grasp_poses);

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
  bool checkGraspPose(int model_index, int direction_index, int orientation_index, int x_index, int z_index);

  /*!
   * detect objects in scene
   */
  bool validateGraspPose(std::vector<int> contact_index,
                         pcl::PointCloud<PointType>::Ptr model_aligned,
                         pcl::PointCloud<PointType>::Ptr normals_aligned);

  /*!
   * detect objects in scene
   */
  bool showQuaternions();

  /*!
   * detect objects in scene
   */
  bool setModelPath(std::string model_path);

  /*!
   * detect objects in scene
   */
  bool setSavePath(std::string save_path);

  /*!
   * detect objects in scene
   */
  bool setGripperMask(std::vector<finger_data> gripper_mask);

  /*!
   * detect objects in scene
   */
  bool setLeafSize(double leaf_size);

  /*!
   * detect objects in scene
   */
  bool setNumberOfEquatorPoints(int number_of_equator_points);

  /*!
   * detect objects in scene
   */
  bool setMinGraspPoseQuality(double min_grasp_pose_quality);

  /*!
   * detect objects in scene
   */
  bool setNormalSearchRadius(double normal_search_radius);

  /*!
   * detect objects in scene
   */
  bool setNumberOfXSteps(int number_of_x_steps);

  /*!
   * detect objects in scene
   */
  bool setNumberOfZSteps(int number_of_z_steps);

  /*!
   * detect objects in scene
   */
  bool setXIncrement(double x_increment);

  /*!
   * detect objects in scene
   */
  bool setZIncrement(double z_increment);

  void saveGraspPoses(std::string save_file_name, const std::vector<grasp_pose> &grasp_poses);

 private:

  std::string model_path_;
  std::string save_path_;
  std::vector<std::string> models_to_detect_;
  std::vector<finger_data> gripper_mask_;
  std::vector<Eigen::Vector2i> pinch_groups_;
  std::vector<object_data> models_;

  std::vector<Eigen::Vector3d> grasp_directions_;
  std::vector<double> theta_;
  std::vector<double> phi_;

  std::vector<grasp_pose> grasp_poses_;
  int number_of_pinch_grasp_poses_;
  int number_of_normal_grasp_poses_;

  // Parameters
  double normal_search_radius_;
  double leaf_size_;
  int number_of_equator_points_;  //ammount of points on the meridian of the geodesic grid
                                  //computation time scales with n³.
  double min_grasp_pose_quality_;
  int number_of_x_steps_;
  int number_of_z_steps_;
  double x_increment_;
  double z_increment_;


};

} /* namespace */
