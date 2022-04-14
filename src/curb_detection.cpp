/**
 * @author Seongchang Park (scsc1125@gmail.com)
 * @date 2022-04-14 19:54
 */
#include <lidar_curb_detection/curb_detection.h>

CurbDetector::CurbDetector()
{
  bool init_result = init();
  ROS_ASSERT(init_result);

  theta_r = 180 * M_PI/ 180;
  num_of_channel = 0;
}
CurbDetector::~CurbDetector()
{
}

// Init function
bool CurbDetector::init()
{
  origin_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/origin_output", 10);
  test_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/test", 10);
  os1_sub_ = nh_.subscribe("/input", 10, &CurbDetector::OS1PointCloudCallback, this);

  left_curb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/left_curb", 10);
  right_curb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/right_curb", 10);

  left_curb_pcl1_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/left_pcl1_curb", 10);
  right_curb_pcl1_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/right_pcl1_curb", 10);

  left_curb_udp_pub_ = nh_.advertise<lidar_msgs::Curb>("/left_curb_udp", 10);
  right_curb_udp_pub_ = nh_.advertise<lidar_msgs::Curb>("/right_curb_udp", 10);

  return true;
}

// Callback 함수 for OS1 point cloud msgs
void CurbDetector::OS1PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs)
{
  static double prev = 0, cur;
  cur = ros::Time::now().toSec();

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr curb_candatate_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*cloud_msgs, *cloud_ptr);

  cloud_test_ptr->header = cloud_ptr->header;
  curb_candatate_ptr->header = cloud_ptr->header;
  num_of_channel = 20;

  cloud_test_ptr->points.resize (1024*num_of_channel);
  int idx = 0;
  int cur_channel = 0;

  for(int i = cloud_ptr->size() - 1; i >= 0; i--)
  {
    cloud_ptr->points[i].z += OS1_HEIGHT;

    if ((i + 1) % 1024 == 0)
    {
      cur_channel++;
    }

    if (cur_channel <= num_of_channel)
    {
      cloud_test_ptr->points[idx].x = cloud_ptr->points[i].x;
      cloud_test_ptr->points[idx].y = cloud_ptr->points[i].y;
      cloud_test_ptr->points[idx].z = cloud_ptr->points[i].z;
      cloud_test_ptr->points[idx].intensity = cloud_ptr->points[i].intensity;
      idx++;
    }
  }

  int step = 1;
  float points_dist = 0.0;  // m
  float points_rad = 0.0;   // rad
  float points_deg = 0.0;   // deg
  int count = 0;
  for (int ch = 0; ch < num_of_channel; ch++)
  {
    for (int p = 0; p < 1024 - step; p++)
    {
      points_dist = sqrt(pow(cloud_test_ptr->points[1024*ch+p].x - cloud_test_ptr->points[1024*ch+p+step].x,2)+ 
      pow(cloud_test_ptr->points[1024*ch+p].y - cloud_test_ptr->points[1024*ch+p+step].y,2));

      points_rad = abs(atan2(cloud_test_ptr->points[1024*ch+p].z - cloud_test_ptr->points[1024*ch+p+step].z, cloud_test_ptr->points[1024*ch+p].y - cloud_test_ptr->points[1024*ch+p+step].y));
      points_deg = points_rad * 180 / M_PI;
      
      if (points_deg < 0) points_deg += 360;

      if (points_deg >= 50 && points_deg <= 130 && points_dist < 1)
      {
        curb_candatate_ptr->points.push_back(cloud_test_ptr->points[1024*ch+p]);
      }
      count++;
    }
  }
  
  if (curb_candatate_ptr->size() > 0)
  {   
    // Input point cloud, filter resolution, min Point, max Point
    constexpr float kFilterResolution = 0.2;    // 0.2 -> voxel leaf size
    //                               x    y     z
    const Eigen::Vector4f kMinPoint(0, -15, -1, 1);  // -50 -6 -3 1
    const Eigen::Vector4f kMaxPoint(60, 15, 0.25, 1);     // 60 6.5 4 1
    auto filter_cloud = FilterCloud(curb_candatate_ptr, kFilterResolution, kMinPoint, kMaxPoint);

    // ##################################
    //  Seperate Left / Right
    // ##################################
    pcl::PointCloud<pcl::PointXYZI>::Ptr left_candidate_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr right_candidate_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    left_candidate_ptr->header = filter_cloud->header;
    right_candidate_ptr->header = filter_cloud->header;
    
    for (int i = 0; i < filter_cloud->size(); i++)
    {
      // Set height = 0 at each z value
      filter_cloud->points[i].z = 0;

      // left if not right
      if (filter_cloud->points[i].y >= 0)
        left_candidate_ptr->points.push_back(filter_cloud->points[i]);
      else
        right_candidate_ptr->points.push_back(filter_cloud->points[i]);
    }

    // #################
    //  RANSAC
    // #################
    auto left_curb = RANSACCloud(left_candidate_ptr);
    auto right_curb = RANSACCloud(right_candidate_ptr);

    cout << "num left curb: " << left_curb->size() << endl;
    cout << "num right curb: " << right_curb->size() << endl;

    auto l_msg = lidar_msgs::Curb();
    auto r_msg = lidar_msgs::Curb();
    
    for (size_t i = 0; i < 100; i++)
    {
      if(i < left_curb->size())
      {
        l_msg.points[i].x = left_curb->points[i].x;
        l_msg.points[i].y = left_curb->points[i].y;
        l_msg.points[i].z = left_curb->points[i].z;
        l_msg.points[i].intensity = left_curb->points[i].intensity;
      }
      else
      {
        l_msg.points[i].x = 0;
        l_msg.points[i].y = 0;
        l_msg.points[i].z = 0;
        l_msg.points[i].intensity = -1;
      }
      if(i < right_curb->size())
      {
        r_msg.points[i].x = right_curb->points[i].x;
        r_msg.points[i].y = right_curb->points[i].y;
        r_msg.points[i].z = right_curb->points[i].z;
        r_msg.points[i].intensity = right_curb->points[i].intensity;
      }
      else
      {
        r_msg.points[i].x = 0;
        r_msg.points[i].y = 0;
        r_msg.points[i].z = 0;
        r_msg.points[i].intensity = -1;
      }
    }
    l_msg.header.stamp = ros::Time::now();
    r_msg.header.stamp = ros::Time::now();

    left_curb_udp_pub_.publish(l_msg);
    right_curb_udp_pub_.publish(r_msg);
    
    // For Test
    sensor_msgs::PointCloud2 test_cloud;
    pcl::toROSMsg(*cloud_test_ptr, test_cloud);
    cloud_test_ptr->clear();
    test_pub_.publish(test_cloud);

    // For Origin Data
    sensor_msgs::PointCloud2 origin_point_cloud;
    pcl::toROSMsg(*cloud_ptr, origin_point_cloud);

    // For left curb detection
    pcl::toROSMsg(*left_curb, ros_left_curb_);

    // For left curb detection
    pcl::toROSMsg(*right_curb, ros_right_curb_);

    // Publish message
    origin_pub_.publish(origin_point_cloud);
    left_curb_pub_.publish(ros_left_curb_);
    right_curb_pub_.publish(ros_right_curb_);

    prev = ros::Time::now().toSec();
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CurbDetector::RANSACCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  // #################
  //  RANSAC
  // #################
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_points        (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_points_neg    (new pcl::PointCloud<pcl::PointXYZI>);

  // Object for Line fitting
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr      inliers      (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
  seg.setInputCloud (cloud);                //입력 
  seg.setModelType (pcl::SACMODEL_LINE);    //적용 모델  // Configure the object to look for a plane.
  seg.setMethodType (pcl::SAC_RANSAC);      //적용 방법   // Use RANSAC method.
  seg.setMaxIterations (1000);              //최대 실행 수
  seg.setDistanceThreshold (0.01);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
  //seg.setRadiusLimits(0, 0.1);            // cylinder경우, Set minimum and maximum radii of the cylinder.
  seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 
  
  pcl::copyPointCloud<pcl::PointXYZI> (*cloud, *inliers, *inlier_points);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (inlier_points);
  extract.setIndices (inliers);
  extract.setNegative (true);//false
  extract.filter (*inlier_points_neg);

  return inlier_points_neg;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CurbDetector::FilterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  // Filter point cloud that is out of region of interest
  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloud);        // filtered_cloud
  region.filter(*filtered_cloud);     // filtered_cloud

  std::vector<int> indices;
  
  // Filter point cloud on the roof of host vehicle
  region.setMin(Eigen::Vector4f(-2, -0.9, -1.9, 1));  // -1.5 1.7 -1 1
  region.setMax(Eigen::Vector4f(2.2, 0.9, 0, 1));  // 2.6 1.7 -0.4 1
  region.setInputCloud(filtered_cloud);   // filtered_cloud
  region.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for (int index : indices)
  {
      inliers->indices.push_back(index);
  }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  // Extract the point cloud on roof
  extract.setInputCloud(filtered_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true
  extract.filter(*filtered_cloud);

  return filtered_cloud;
}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> CurbDetector::SeparateClouds(
        const pcl::PointIndices::Ptr& inliers, const  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  // Copy inliers point cloud as plane
  for (int index : inliers->indices)
  {
      plane_cloud->points.push_back(cloud->points[index]);
  }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  // Extract the inliers so that we can get obstacles point cloud
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(obstacle_cloud, plane_cloud);
  return segResult;
}

// Use the pair object to hold your segmented results for the obstacle point cloud and the road point cloud
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> CurbDetector::SegmentPlane
(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int maxIterations, float distanceThreshold)
{
	// Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  // Segment the largest planar component from the input cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
      std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
  }

  std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = SeparateClouds(inliers, cloud);
  return segResult;
}