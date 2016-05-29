/*
 * GraspPoseDetection.cpp
 *
 *  Created on: May 26, 2016
 *      Author: yves
 */

#include <boost/thread/thread.hpp>
#include <math.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/board.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include "grasp_pose_detection/GraspPoseDetection.hpp"

std::stack<clock_t> tictoc_stack;

void tic()
{
  tictoc_stack.push(clock());
}

void toc()
{
  std::cout << "Time elapsed for object localisation: "
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

/*!
 * detect objects in scene
 */
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
    grasp_directions_.push_back(grasp_direction);
  }

  while (phi_latitude <= M_PI / 2) {

    //calculate ammount of points and  angle
    n_latitude = ceil(number_of_equator_points_ * cos(phi_latitude));
    theta_latitude = 2 * M_PI / n_latitude;

    //save grasp_directions
    for (int i = 0; i < n_latitude; i++) {
      Eigen::Vector3d grasp_direction;
      grasp_direction.x() = cos(phi_latitude) * cos(theta_latitude * i);
      grasp_direction.y() = cos(phi_latitude) * sin(theta_latitude * i);
      grasp_direction.z() = sin(phi_latitude);
      grasp_directions_.push_back(grasp_direction);
    }

    // set phi of next latitude
    phi_latitude = phi_latitude + theta_equator;
  }
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

