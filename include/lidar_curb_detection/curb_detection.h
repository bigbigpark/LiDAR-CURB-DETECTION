/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-14 19:43
 */
#pragma once

#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>

// PCL for transformation
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

// PCL specific includes
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <ros/ros.h>
#include <vector>
#include <ctime>
#include <time.h>
#include <chrono>
#include <cmath>

#include <lidar_msgs/Curb.h>

using namespace std;

#define OS1_HEIGHT 1.9

class CurbDetector
{
public:
  CurbDetector();
  ~CurbDetector();
public:
  bool init();

public:
  void OS1PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs);
  pcl::PointCloud<pcl::PointXYZI>::Ptr FilterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint);
  pcl::PointCloud<pcl::PointXYZI>::Ptr RANSACCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> SeparateClouds(const pcl::PointIndices::Ptr& inliers, const  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int maxIterations, float distanceThreshold);

public:
  int num_of_channel;

private:
  ros::NodeHandle nh_;
  
  ros::Publisher origin_pub_;
  ros::Publisher test_pub_;

  ros::Publisher left_curb_pub_;
  ros::Publisher right_curb_pub_;

  ros::Subscriber os1_sub_;

  sensor_msgs::PointCloud2 ros_left_curb_;
  sensor_msgs::PointCloud2 ros_right_curb_;

  float theta_r;

  ros::Publisher left_curb_pcl1_pub_;
  ros::Publisher right_curb_pcl1_pub_;

  ros::Publisher left_curb_udp_pub_;
  ros::Publisher right_curb_udp_pub_;
};