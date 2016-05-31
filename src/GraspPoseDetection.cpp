/*
 * GraspPoseDetection.cpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#include <boost/thread/thread.hpp>
#include <math.h>
#include <string>
#include <limits.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>

#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "grasp_pose_detection/GraspPoseDetection.hpp"

std::stack<clock_t> tictoc_stack;

void tic()
{
  tictoc_stack.push(clock());
}

void toc()
{
  std::cout << "Time elapsed for grasp pose detection: "
            << ((double) (clock() - tictoc_stack.top())) / CLOCKS_PER_SEC << "s" << std::endl;
  tictoc_stack.pop();
}

namespace grasp_pose_detection {

GraspPoseDetection::GraspPoseDetection(std::vector<std::string>& models_to_detect)
    : normal_search_radius_(0.01),
      leaf_size_(0.002),
      number_of_equator_points_(100),
      min_grasp_pose_quality_(0.5),
      number_of_normal_grasp_poses_(0),
      number_of_pinch_grasp_poses_(0),
      number_of_x_steps_(0),
      number_of_z_steps_(0),
      x_increment_(0.005),
      z_increment_(0.005),
      use_collision_detection_(true),
      use_pinch_(true)
{
  models_to_detect_ = models_to_detect;
  models_.resize(models_to_detect.size());
}

GraspPoseDetection::~GraspPoseDetection()
{
}

bool GraspPoseDetection::detectGraspPose(std::vector<std::string>& models_detected,
                                         std::vector<int>& number_of_grasp_poses)
{
  tic();
  // Load and Prepare Models
  loadModelData();
  ROS_INFO("Models are loaded.");
  DownSample();
  ROS_INFO_STREAM("Down sampled with leaf size: " << leaf_size_ << ".");
  computeNormals();
  ROS_INFO("Normals are computed.");

  // Compute Grasp Directions using a pseudo geodesic grid
  // (could use real geodesic grid like ISS but this is faster and good enough)

  computeGraspDirections();
  ROS_INFO_STREAM("Computed " << grasp_directions_.size() << " grasp directions.");
  ROS_INFO_STREAM(
      "Trying " << number_of_equator_points_ * grasp_directions_.size() << " grasp poses");

  // detect the grasp poses by iterating through the grasp directions
  for (int model_index = 0; model_index < models_.size(); model_index++) {
    for (int direction_index = 0; direction_index < grasp_directions_.size(); direction_index++) {
      for (int orientation_index = 0; orientation_index < number_of_equator_points_;
          orientation_index++) {
        for (int x_index = -number_of_x_steps_ / 2; x_index <= number_of_x_steps_ / 2; x_index++) {
          for (int z_index = -number_of_z_steps_ / 2; z_index <= number_of_z_steps_ / 2;
              z_index++) {
            checkGraspPose(model_index, direction_index, orientation_index, x_index, z_index);
          }
        }
      }
    }
    if (number_of_normal_grasp_poses_ + number_of_pinch_grasp_poses_ > 0) {
      number_of_grasp_poses.push_back(number_of_normal_grasp_poses_ + number_of_pinch_grasp_poses_);
      models_detected.push_back(models_to_detect_[model_index]);
    }

    ROS_INFO_STREAM("Model: " << models_to_detect_[model_index]);
    ROS_INFO_STREAM("Detected " << number_of_normal_grasp_poses_ << " normal grasp_poses.");
    ROS_INFO_STREAM("Detected " << number_of_pinch_grasp_poses_ << " pinched grasp_poses.");

    std::string saveFile = save_path_ + models_to_detect_[model_index];
    saveFile = saveFile + "_grasp_poses";
    saveFile = saveFile + ".gp";
    saveGraspPoses(saveFile, grasp_poses_);

    //showQuaternions();
    grasp_poses_.clear();
    number_of_normal_grasp_poses_ = 0;
    number_of_pinch_grasp_poses_ = 0;

  }

  // stop timer
  toc();
  return true;
}

bool GraspPoseDetection::loadModelData()
{

  for (int i = 0; i < models_to_detect_.size(); i++) {
    std::string model_file_name = model_path_ + models_to_detect_[i];
    model_file_name = model_file_name + ".pcd";

    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(model_file_name, *point_cloud_ptr) == -1) {
      PCL_ERROR("Couldn't read input file base \n");
      return (-1);
    }

    ROS_INFO_STREAM("Model " << models_to_detect_[i] << " loaded.");
    models_[i].point_cloud_ptr = point_cloud_ptr;
  }
  return true;
}

bool GraspPoseDetection::computeNormals()
{

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointType, pcl::Normal> n;
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

  for (int i = 0; i < models_.size(); i++) {
    tree->setInputCloud(models_[i].point_cloud_ptr);
    n.setInputCloud(models_[i].point_cloud_ptr);
    n.setSearchMethod(tree);
    n.setRadiusSearch(normal_search_radius_);
    n.compute(*normals);
    models_[i].normals = normals;

    pcl::PointCloud<pcl::PointXYZ>::Ptr normals_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int j = 0; j < normals->size(); j++) {
      pcl::PointXYZ normal_point;
      normal_point.x = normals->points[j].normal_x;
      normal_point.y = normals->points[j].normal_y;
      normal_point.z = normals->points[j].normal_z;
      normals_cloud->push_back(normal_point);
    }
    models_[i].normals_cloud = normals_cloud;
  }
  return true;
}

bool GraspPoseDetection::DownSample()
{
  pcl::PointCloud<PointType>::Ptr cloud_downsampled(new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> sor;

  for (int i = 0; i < models_.size(); i++) {
    sor.setInputCloud(models_[i].point_cloud_ptr);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*cloud_downsampled);
    *models_[i].point_cloud_ptr = *cloud_downsampled;
  }

  return true;
}

bool GraspPoseDetection::computeGraspDirections()
{
  double theta_equator = 2 * M_PI / number_of_equator_points_;

  double n_latitude;
  double theta_latitude;
  double phi_latitude = theta_equator;

  //save grasp_directions of equator
  for (int i = 0; i < number_of_equator_points_; i++) {
    Eigen::Vector3d grasp_direction;
    grasp_direction.x() = cos(theta_equator * i);
    grasp_direction.y() = sin(theta_equator * i);
    grasp_direction.z() = 0;
    theta_.push_back(theta_equator * i);
    phi_.push_back(0.0);
    grasp_directions_.push_back(grasp_direction);
  }

  while (phi_latitude <= M_PI / 2) {
    //calculate ammount of points and  angle
    n_latitude = ceil(number_of_equator_points_ * cos(phi_latitude));
    theta_latitude = 2 * M_PI / n_latitude;

    //save grasp_directions
    for (int i = 0; i < n_latitude; i++) {
      Eigen::Vector3d grasp_direction;

      // positive hemisphere
      grasp_direction.x() = cos(phi_latitude) * cos(theta_latitude * i);
      grasp_direction.y() = cos(phi_latitude) * sin(theta_latitude * i);
      grasp_direction.z() = sin(phi_latitude);

      theta_.push_back(theta_latitude * i);
      phi_.push_back(phi_latitude);
      grasp_directions_.push_back(grasp_direction);

      // negative hemisphere
      grasp_direction.x() = cos(phi_latitude) * cos(theta_latitude * i);
      grasp_direction.y() = cos(phi_latitude) * sin(theta_latitude * i);
      grasp_direction.z() = -sin(phi_latitude);

      theta_.push_back(theta_latitude * i);
      phi_.push_back(-phi_latitude);
      grasp_directions_.push_back(grasp_direction);
    }

    // set phi of next latitude
    phi_latitude = phi_latitude + theta_equator;
  }
  return true;
}

bool GraspPoseDetection::checkGraspPose(int model_index, int direction_index, int orientation_index,
                                        int x_index, int z_index)
{
  // Get model cloud description in grasp pose frame
  float theta = (float) theta_[direction_index];
  float psi = (float) -( M_PI / 2 - phi_[direction_index]);
  float delta = 2 * M_PI / number_of_equator_points_ * orientation_index;
  float x = x_index * x_increment_;
  float z = z_index * z_increment_;

  Eigen::Matrix4f translation;
  translation << 1, 0, 0, -x, 0, 1, 0, 0, 0, 0, 1, -z, 0, 0, 0, 1;

  Eigen::Matrix4f ori_rot;
  ori_rot << cos(-delta), -sin(-delta), 0, 0, sin(-delta), cos(-delta), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Matrix4f y_rot;
  y_rot << cos(psi), 0, sin(psi), 0, 0, 1, 0, 0, -sin(psi), 0, cos(psi), 0, 0, 0, 0, 1;

  Eigen::Matrix4f z_rot;
  z_rot << cos(-theta), -sin(-theta), 0, 0, sin(-theta), cos(-theta), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Matrix4f tf = translation * ori_rot * y_rot * z_rot;

  pcl::PointCloud<PointType>::Ptr model_aligned(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr normals_aligned(new pcl::PointCloud<PointType>);

  pcl::transformPointCloud(*models_[model_index].point_cloud_ptr, *model_aligned, tf);
  pcl::transformPointCloud(*models_[model_index].normals_cloud, *normals_aligned, tf);

  // get normal angles and finger poses of each finger tip contact

  std::vector<int> contact_index;
  std::vector<double> contact_distance;
  contact_index.resize(gripper_mask_.size());
  contact_distance.resize(gripper_mask_.size(), std::numeric_limits<double>::max());

  for (int i = 0; i < gripper_mask_.size(); i++) {

    // Transform model in trajectory frame to segment the points in the collision path
    double gamma = gripper_mask_[i].trajectory_angle;
    Eigen::Matrix4f traj_rot;
    traj_rot << 1, 0, 0, 0, 0, cos(-gamma), -sin(-gamma), 0, 0, sin(-gamma), cos(-gamma), 0, 0, 0, 0, 1;

    pcl::PointCloud<PointType>::Ptr model_traj_aligned(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*model_aligned, *model_traj_aligned, traj_rot);
    Eigen::Vector4f initial_position;
    initial_position.x() = (float) gripper_mask_[i].initial_position.x();
    initial_position.y() = (float) gripper_mask_[i].initial_position.y();
    initial_position.z() = (float) gripper_mask_[i].initial_position.z();
    initial_position.w() = 1;
    Eigen::Vector4f transformed_initial_position = traj_rot * initial_position;

    // segment points in trajectory
    std::vector<int> inlier_indexes;

    for (int j = 0; j < models_[model_index].point_cloud_ptr->size(); j++) {
      pcl::PointXYZ point = model_traj_aligned->points[j];
      float width = (float) gripper_mask_[i].plate_width;
      float height = (float) gripper_mask_[i].plate_height * fabs(cos(gamma));

      if (point.x >= (transformed_initial_position.x() - (width / 2))
          && point.x <= (transformed_initial_position.x() + (width / 2))
          && point.y >= (transformed_initial_position.y())
          && point.z >= (transformed_initial_position.z() - (height / 2))
          && point.z <= (transformed_initial_position.z() + (height / 2))) {
        inlier_indexes.push_back(j);
      }
    }

    // pick contact point from segmented cloud
    for (int j = 0; j < inlier_indexes.size(); j++) {
      Eigen::Vector3d relative_point_position;
      relative_point_position.x() = model_aligned->points[inlier_indexes[j]].x
          - gripper_mask_[i].initial_position.x();
      relative_point_position.y() = model_aligned->points[inlier_indexes[j]].y
          - gripper_mask_[i].initial_position.y();
      relative_point_position.z() = model_aligned->points[inlier_indexes[j]].z
          - gripper_mask_[i].initial_position.z();

      double point_collision_distance = relative_point_position.dot(gripper_mask_[i].plate_normal);

      if (point_collision_distance < contact_distance[i]) {
        contact_distance[i] = point_collision_distance;
        contact_index[i] = inlier_indexes[j];
      }
    }

    // collision check (works only if trajectory has no x component)
    if (use_collision_detection_) {
      for (int j = 0; j < model_aligned->size(); j++) {
        double relative_y_position = model_aligned->points[j].y
            - model_aligned->points[contact_index[i]].y;

        double penetration_distance = -relative_y_position
            * (gripper_mask_[i].plate_normal.y() / fabs(gripper_mask_[i].plate_normal.y()));

        double z_movement = tan(gripper_mask_[i].trajectory_angle)
            * (model_aligned->points[contact_index[i]].y - gripper_mask_[i].initial_position.y());

        if (model_aligned->points[j].x
            >= (gripper_mask_[i].initial_position.x() - (gripper_mask_[i].plate_width / 2))
            && model_aligned->points[j].x
                <= (gripper_mask_[i].initial_position.x() + (gripper_mask_[i].plate_width / 2))
            && penetration_distance > gripper_mask_[i].collision_distance
            && model_aligned->points[j].z
                >= (gripper_mask_[i].initial_position.z() + z_movement + (gripper_mask_[i].plate_height / 2))) {
          return true;
        }
      }
    }
  }

// Validation of grasp pose
  if (validateGraspPose(contact_index, model_aligned, normals_aligned)) {
    Eigen::Matrix4f grasp_transform = tf.inverse();
    Eigen::Matrix3f rotation = grasp_transform.block<3, 3>(0, 0);
    Eigen::Vector3f translation = grasp_transform.block<3, 1>(0, 3);

    // Convert homogenous transformation matrix to pose
    geometry_msgs::Pose new_pose;

    new_pose.position.x = translation[0];
    new_pose.position.y = translation[1];
    new_pose.position.z = translation[2];

    Eigen::Quaternionf quat(rotation);
    new_pose.orientation.w = quat.w();
    new_pose.orientation.x = quat.x();
    new_pose.orientation.y = quat.y();
    new_pose.orientation.z = quat.z();
    grasp_poses_.back().grasp_pose = new_pose;
  }

  return true;
}

bool GraspPoseDetection::validateGraspPose(std::vector<int> contact_index,
                                           pcl::PointCloud<PointType>::Ptr model_aligned,
                                           pcl::PointCloud<PointType>::Ptr normals_aligned)
{
  std::vector<Eigen::Vector3d> contact_normals;
  std::vector<bool> contact_grip_ensured;
  std::vector<double> contact_quality;

  contact_normals.resize(gripper_mask_.size());
  contact_grip_ensured.resize(gripper_mask_.size(), false);
  contact_quality.resize(gripper_mask_.size());

// building the quality criteria
  for (int i = 0; i < gripper_mask_.size(); i++) {
    contact_normals[i].x() = normals_aligned->points[contact_index[i]].x;
    contact_normals[i].y() = normals_aligned->points[contact_index[i]].y;
    contact_normals[i].z() = normals_aligned->points[contact_index[i]].z;
    contact_normals[i].normalize();

    contact_quality[i] = contact_normals[i].dot(gripper_mask_[i].plate_normal);

    if (contact_quality[i] >= cos(gripper_mask_[i].max_grasp_angle)) {
      contact_grip_ensured[i] = true;
    }
  }

  bool grasp_pose_ensured = true;
  double grasp_pose_quality = 1;

  for (int i = 0; i < gripper_mask_.size(); i++) {
    grasp_pose_ensured = grasp_pose_ensured && contact_grip_ensured[i];
    grasp_pose_quality = grasp_pose_quality * contact_quality[i];
  }

// selection using the quality values
  if (grasp_pose_ensured && grasp_pose_quality > min_grasp_pose_quality_) {
    grasp_pose new_grasp_pose;
    new_grasp_pose.grasp_pose_quality = grasp_pose_quality;
    for (int i = 0; i < gripper_mask_.size(); i++) {
      new_grasp_pose.finger_position.push_back(model_aligned->points[contact_index[i]].y);
    }
    grasp_poses_.push_back(new_grasp_pose);
    number_of_normal_grasp_poses_++;
    return true;
  } else if (pinch_groups_.size() > 0 && use_pinch_) {

    // checking if pinch grip is good enough for each pinch group and reset contact_quality
    for (int i = 0; i < pinch_groups_.size(); i++) {
      // projecting contact normals on yz-plane to get the influence of x direction out
      Eigen::Vector3d projected_contact_normal_x = contact_normals[pinch_groups_[i].x()];
      Eigen::Vector3d projected_contact_normal_y = contact_normals[pinch_groups_[i].y()];
      projected_contact_normal_x.x() = 0.0;
      projected_contact_normal_y.x() = 0.0;
      projected_contact_normal_x.normalize();
      projected_contact_normal_y.normalize();

      double projected_contact_quality_x = projected_contact_normal_x.dot(
          gripper_mask_[pinch_groups_[i].x()].plate_normal);
      double projected_contact_quality_y = projected_contact_normal_y.dot(
          gripper_mask_[pinch_groups_[i].y()].plate_normal);

      if ((contact_normals[pinch_groups_[i].x()].x() > 0)
          != (contact_normals[pinch_groups_[i].y()].x() > 0)
          && projected_contact_quality_x >= cos(gripper_mask_[pinch_groups_[i].x()].max_grasp_angle)
          && projected_contact_quality_y
              >= cos(gripper_mask_[pinch_groups_[i].y()].max_grasp_angle)) {
        contact_quality[pinch_groups_[i].x()] = projected_contact_quality_x;
        contact_quality[pinch_groups_[i].y()] = projected_contact_quality_y;
        contact_grip_ensured[pinch_groups_[i].x()] = true;
        contact_grip_ensured[pinch_groups_[i].y()] = true;
      }
    }
    // checking grasp_pose_quality for pinched grasp
    grasp_pose_ensured = true;
    grasp_pose_quality = 1;
    for (int j = 0; j < gripper_mask_.size(); j++) {
      grasp_pose_ensured = grasp_pose_ensured && contact_grip_ensured[j];
      grasp_pose_quality = grasp_pose_quality * contact_quality[j];
    }
    // selection using the quality values
    if (grasp_pose_ensured && grasp_pose_quality > min_grasp_pose_quality_) {
      grasp_pose new_grasp_pose;
      new_grasp_pose.grasp_pose_quality = grasp_pose_quality;
      for (int i = 0; i < gripper_mask_.size(); i++) {
        new_grasp_pose.finger_position.push_back(model_aligned->points[contact_index[i]].y);
      }
      number_of_pinch_grasp_poses_++;
      grasp_poses_.push_back(new_grasp_pose);
      return true;
    }
  }
  return false;
}

bool GraspPoseDetection::showQuaternions()
{

  Eigen::Matrix3f rotation;
  rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;

// Convert homogenous transformation matrix to pose
  geometry_msgs::Pose new_pose;

  Eigen::Quaternionf quat(rotation);
  new_pose.orientation.w = quat.w();
  new_pose.orientation.x = quat.x();
  new_pose.orientation.y = quat.y();
  new_pose.orientation.z = quat.z();

  std::cout << "Unit Quaternion: " << new_pose.orientation.x << " " << new_pose.orientation.y << " "
            << new_pose.orientation.z << " " << new_pose.orientation.w << " " << std::endl;

  for (int i = 0; i < grasp_poses_.size(); i++) {
    std::cout << "Grasp pose :" << i << std::endl;

    std::cout << "position (x,y,z): " << grasp_poses_[i].grasp_pose.position.x << " "
        << grasp_poses_[i].grasp_pose.position.y << " " << grasp_poses_[i].grasp_pose.position.z
        << " " << std::endl;

    std::cout << "orientation (x,y,z,w): " << grasp_poses_[i].grasp_pose.orientation.x << " "
        << grasp_poses_[i].grasp_pose.orientation.y << " "
        << grasp_poses_[i].grasp_pose.orientation.z << " "
        << grasp_poses_[i].grasp_pose.orientation.w << " " << std::endl;

    std::cout << "grasp_poses_finger_pos = " << grasp_poses_[i].finger_position[0] << std::endl;
    std::cout << "grasp_poses_finger_pos = " << grasp_poses_[i].finger_position[1] << std::endl;
    std::cout << "grasp_poses_finger_pos = " << grasp_poses_[i].finger_position[2] << std::endl;

    std::cout << "grasp_poses_quality = " << grasp_poses_[i].grasp_pose_quality << std::endl;
  }
  return true;

}

bool GraspPoseDetection::setModelPath(std::string model_path)
{
  model_path_ = model_path;
  return true;
}

bool GraspPoseDetection::setSavePath(std::string save_path)
{
  save_path_ = save_path;
  return true;
}

bool GraspPoseDetection::setGripperMask(std::vector<finger_data> gripper_mask)
{
  gripper_mask_ = gripper_mask;
  int number_of_pinch_groups;

  for (int i = 0; i < gripper_mask_.size(); i++) {
    gripper_mask_[i].plate_normal.normalize();
    if (gripper_mask_[i].pinch_group >= 0) {
      number_of_pinch_groups = gripper_mask_[i].pinch_group + 1;
    }
  }
  pinch_groups_.resize(number_of_pinch_groups);
  for (int i = 0; i < gripper_mask_.size(); i++) {
    if (gripper_mask_[i].pinch_group >= 0) {
      if (pinch_groups_[gripper_mask_[i].pinch_group].x() > gripper_mask_.size()) {
        pinch_groups_[gripper_mask_[i].pinch_group].x() = i;
      } else {
        pinch_groups_[gripper_mask_[i].pinch_group].y() = i;
      }
    }
  }
  return true;
}

bool GraspPoseDetection::setLeafSize(double leaf_size)
{
  leaf_size_ = leaf_size;
  return true;
}

bool GraspPoseDetection::setNumberOfEquatorPoints(int number_of_equator_points)
{
  number_of_equator_points_ = number_of_equator_points;
  if ((number_of_equator_points % 4) != 0)
    ROS_WARN(
        "Consider using a multiple of 4 as 'number_of_equator_points' to produce better pseudo geodesic grids.");
  return true;
}

bool GraspPoseDetection::setMinGraspPoseQuality(double min_grasp_pose_quality)
{
  min_grasp_pose_quality_ = min_grasp_pose_quality;
  return true;
}

bool GraspPoseDetection::setNormalSearchRadius(double normal_search_radius)
{
  normal_search_radius_ = normal_search_radius;
  return true;
}

bool GraspPoseDetection::setNumberOfXSteps(int number_of_x_steps)
{
  if (number_of_x_steps % 2 != 0) {
    ROS_WARN("Use even numbers of steps! number_of_x_steps will be increased by 1.");
    number_of_x_steps++;
  }
  number_of_x_steps_ = number_of_x_steps;
  return true;
}

bool GraspPoseDetection::setNumberOfZSteps(int number_of_z_steps)
{
  if (number_of_z_steps % 2 != 0) {
    ROS_WARN("Use even numbers of steps! number_of_z_steps will be increased by 1.");
    number_of_z_steps++;
  }
  number_of_z_steps_ = number_of_z_steps;
  return true;
}

bool GraspPoseDetection::setXIncrement(double x_increment)
{
  x_increment_ = x_increment;
  return true;
}

bool GraspPoseDetection::setZIncrement(double z_increment)
{
  z_increment_ = z_increment;
  return true;
}


bool GraspPoseDetection::setUseCollisionDetection(bool use_collision_detection){
  use_collision_detection_ = use_collision_detection;
  return true;
}

bool GraspPoseDetection::setUsePinch(bool use_pinch){
  use_pinch_ = use_pinch;
  return true;
}

void GraspPoseDetection::saveGraspPoses(std::string save_file_name,
                                        const std::vector<grasp_pose> &grasp_poses)
{
  std::cout << "save grasp poses to: " << save_file_name << std::endl;
  std::ofstream out(save_file_name, std::ios::out | std::ios::trunc);

  typename std::vector<grasp_pose>::size_type size1 = grasp_poses.size();
  out.write((char*) &size1, sizeof(size1));

  for (int i = 0; i < size1; i++) {
    // write finger position vector
    typename std::vector<double>::size_type size2 = grasp_poses[i].finger_position.size();
    out.write((char*) &size2, sizeof(size2));
    out.write((char*) &grasp_poses[i].finger_position[0],
              grasp_poses[i].finger_position.size() * sizeof(double));

    // write pose
    out.write((char*) &grasp_poses[i].grasp_pose, sizeof(geometry_msgs::Pose));
    out.write((char*) &grasp_poses[i].grasp_pose_quality, sizeof(double));
  }
  out.close();
}

} /* namespace grasp_pose_detection*/

