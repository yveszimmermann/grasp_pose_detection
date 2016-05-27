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

void tic() { tictoc_stack.push(clock()); }

void toc() {
  std::cout << "Time elapsed for object localisation: "
            << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC << "s"
            << std::endl;
  tictoc_stack.pop();
}

namespace grasp_pose_detection {

GraspPoseDetection::GraspPoseDetection(std::vector<std::string>& models_to_detect, std::vector<std::string>& models_detected, std::vector<int>& number_of_grasp_poses) {

  std::cout << "function entered" << std::endl;
  for (int i = 0; i < models_to_detect.size(); i++){
    models_detected.push_back(models_to_detect[i]);
    number_of_grasp_poses.push_back(i);
  }

}

GraspPoseDetection::~GraspPoseDetection() {}

bool GraspPoseDetection::detectGraspPose() {

  return true;
}


} /* namespace grasp_pose_detection*/

