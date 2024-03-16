#ifndef LIDAR_OBJECT_DETECTOR_H
#define LIDAR_OBJECT_DETECTOR_H

#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include "../lidar_filter/filters.h"

struct BBox
{
    Eigen::Vector3f position;
    Eigen::Vector3f dimension;
    Eigen::Quaternionf quaternion;


	  BBox(Eigen::Vector3f position, Eigen::Vector3f dimension, Eigen::Quaternionf quaternion)
		: position(position), dimension(dimension), quaternion(quaternion)
	  {}

    BBox(Eigen::Vector3f position, Eigen::Vector3f dimension)
		: position(position), dimension(dimension)
	  {}

};

class LidarObjectDetector{

  private:
   StatisticalOutlierRemoval outlierRemovalFilter{50, 1.0};
   DownsampleFilter Downsample{0.1f};
   CropBoxFilter crop{Eigen::Vector4f (-100.0f, -5.0f, -10.0f, 1.0f),Eigen::Vector4f (100, 10, 10, 1)};

  public:
   
   void filter_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
   void segment_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_plane,  pcl::PointCloud<pcl::PointXYZI>::Ptr& objects);
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud, float cluster_tolerance, int min_size, int max_size);   
   BBox ConstructBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster);
   BBox ConstructBoundingBox_PCA(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster);
   std::vector<BBox> GetBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters);
   std::vector<BBox> get_detections(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& segmented_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_plane,pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacles_cloud);
  
};

#endif // LIDAR_OBJECT_DETECTOR_H
