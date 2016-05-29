/*
 * GraspPoseDetection.cpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#include <boost/thread/thread.hpp>
#include <math.h>
#include <string>

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

GraspPoseDetection::GraspPoseDetection(std::vector<std::string>& models_to_detect,
                                       std::vector<std::string>& models_detected,
                                       std::vector<int>& number_of_grasp_poses)
    : normal_search_radius_(0.01),
      normal_angle_threshold_(0.35),
      leaf_size_(0.002),
      number_of_equator_points_(50)
{
  models_to_detect_ = models_to_detect;
  models_.resize(models_to_detect.size());
}

GraspPoseDetection::~GraspPoseDetection()
{
}

bool GraspPoseDetection::detectGraspPose()
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

  // detect the grasp poses by iterating through the grasp directions
  for (int model_index = 0; model_index < models_.size(); model_index++) {
    for (int direction_index = 0; direction_index < grasp_directions_.size(); direction_index++) {
      // TODO do iteration of height
      for (int orientation_index = 0; orientation_index < number_of_equator_points_;
          orientation_index++) {
        checkGraspPose(model_index, direction_index, orientation_index);
      }
      //TODO keep only the "best orientations of one direction
      //push the best selection of grasp poses to models_detected and number_of_grasp_poses

    }
  }
  std::cout << "Grasp posed detected" << std::endl;
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

bool GraspPoseDetection::checkGraspPose(int model_index, int direction_index, int orientation_index)
{
  // Get model cloud description in grasp pose frame
  float theta = (float) theta_[direction_index];
  float psi = (float) -( M_PI / 2 - phi_[direction_index]);
  float delta = 2 * M_PI / number_of_equator_points_ * orientation_index;

  // TODO convert this ori_rot to a wrist transformation, to allow translation of the gripper towards COG
  Eigen::Matrix4f ori_rot;
  ori_rot << cos(-delta), -sin(-delta), 0, 0, sin(-delta), cos(-delta), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Matrix4f y_rot;
  y_rot << cos(psi), 0, sin(psi), 0, 0, 1, 0, 0, -sin(psi), 0, cos(psi), 0, 0, 0, 0, 1;

  Eigen::Matrix4f z_rot;
  z_rot << cos(-theta), -sin(-theta), 0, 0, sin(-theta), cos(-theta), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  Eigen::Matrix4f rot = ori_rot * y_rot * z_rot;

  pcl::PointCloud<PointType>::Ptr model_alligned(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr normals_alligned(new pcl::PointCloud<PointType>);

  pcl::transformPointCloud(*models_[model_index].point_cloud_ptr, *model_alligned, rot);
  pcl::transformPointCloud(*models_[model_index].normals_cloud, *normals_alligned, rot);

  // get normal angles and finger poses of each finger tip contact

  std::vector<int> contact_index;
  std::vector<double> contact_distance;
  contact_index.resize(gripper_mask_.size(),INFINITY);
  contact_distance.resize(gripper_mask_.size(),INFINITY);

  for (int i = 0; i < gripper_mask_.size(); i++) {

    // Transform model in trajectory frame to segment the points in the collision path
    double gamma = gripper_mask_[i].trajectory_angle;
    Eigen::Matrix4f traj_rot;
    traj_rot << 1, 0, 0, 0, 0, cos(-gamma), -sin(-gamma), 0, 0, sin(-gamma), cos(-gamma), 0, 0, 0, 0, 1;

    pcl::PointCloud<PointType>::Ptr model_traj_aligned(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*model_alligned, *model_traj_aligned, traj_rot);
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

      // TODO
      if( false){
      std::cout << "height " << height << std::endl;
      std::cout << "finger " << i << std::endl;
      std::cout << "x " << point.x << std::endl;
      std::cout << "y " << point.y << std::endl;
      std::cout << "z " << point.z << std::endl;

      std::cout << "x_in " << transformed_initial_position.x() << std::endl;
      std::cout << "y_in " << transformed_initial_position.y() << std::endl;
      std::cout << "z_in " << transformed_initial_position.z() << std::endl;
      }

      if (   point.x >= (transformed_initial_position.x() - (width / 2))
          && point.x <= (transformed_initial_position.x() + (width / 2))
          && point.y >= (transformed_initial_position.y())
          && point.z >= (transformed_initial_position.z() - (height / 2))
          && point.z <= (transformed_initial_position.z() + (height / 2))) {
        inlier_indexes.push_back(j);
      }
    }
    //std::cout << "found " << inlier_indexes.size() << " points in finger " << i << "'s path" << std::endl;
    // pick contact point from segmented cloud
    for (int j = 0; j < inlier_indexes.size(); j++) {
      Eigen::Vector3d relative_point_position;
      relative_point_position.x() = model_alligned->points[inlier_indexes[j]].x
          - gripper_mask_[i].initial_position.x();
      relative_point_position.y() = model_alligned->points[inlier_indexes[j]].y
          - gripper_mask_[i].initial_position.y();
      relative_point_position.z() = model_alligned->points[inlier_indexes[j]].z
          - gripper_mask_[i].initial_position.z();

      double point_collision_distance = relative_point_position.dot(gripper_mask_[i].plate_normal);
      if (point_collision_distance < contact_distance[i]) {
        contact_distance[i] = point_collision_distance;
        contact_index[i] = inlier_indexes[j];
        //std::cout << "contact index  = " << contact_index[i] << std::endl;
      }
    }
  }

  // Validation of grasp pose
  // TODO


  return true;
}

bool GraspPoseDetection::setModelPath(std::string model_path)
{
  model_path_ = model_path;
  return true;
}

bool GraspPoseDetection::setGripperMask(std::vector<finger_data> gripper_mask)
{
  gripper_mask_ = gripper_mask;
  return true;
}

} /* namespace grasp_pose_detection*/

