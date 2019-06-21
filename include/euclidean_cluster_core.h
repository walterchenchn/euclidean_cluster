#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#define LEAF_SIZE 0.1 //定义降采样的leaf size，聚类是一个费时运算，为了减少计算量，我们通常先进行降采样
#define MIN_CLUSTER_SIZE 20
//#define MAX_CLUSTER_SIZE 5000
#define MAX_CLUSTER_SIZE 2000

class EuClusterCore
{

private:
  struct Detected_Obj
  {
    jsk_recognition_msgs::BoundingBox bounding_box_;

    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;
    pcl::PointXYZ centroid_;
  };

  ros::Subscriber sub_point_cloud_;

  ros::Publisher pub_bounding_boxs_;

  std::vector<double> seg_distance_, cluster_distance_;

  std_msgs::Header point_cloud_header_;

  void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);

  void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list);

  void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                       double in_max_cluster_distance, std::vector<Detected_Obj> & obj_list);

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);

public:
  EuClusterCore(ros::NodeHandle &nh);
  ~EuClusterCore();
};