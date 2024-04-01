#ifndef LIDAR_OBJECT_DETECTOR_H
#define LIDAR_OBJECT_DETECTOR_H

#include "../lidar_filter/filters.h"
#include <Eigen/Core>
#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <unordered_set>
#include <vector>

struct BBox
{
  size_t id;
  Eigen::Vector3f position;
  Eigen::Vector3f dimension;
  Eigen::Quaternionf quaternion;

  BBox(size_t id, Eigen::Vector3f position, Eigen::Vector3f dimension, Eigen::Quaternionf quaternion)
    : id(id), position(position), dimension(dimension), quaternion(quaternion)
  {
  }

  BBox(size_t id, Eigen::Vector3f position, Eigen::Vector3f dimension)
    : id(id), position(position), dimension(dimension), quaternion(Eigen::Quaternionf::Identity())
  {
  }

  // Default constructor
  BBox()
    : id(0)
    , position(Eigen::Vector3f::Zero())
    , dimension(Eigen::Vector3f::Zero())
    , quaternion(Eigen::Quaternionf::Identity())
  {
  }
};

class LidarObjectDetector
{
private:
  StatisticalOutlierRemoval outlierRemovalFilter{ 50, 1.0 };
  DownsampleFilter Downsample{ 0.1f };
  CropBoxFilter crop{ Eigen::Vector4f(-100.0f, -5.0f, -10.0f, 1.0f), Eigen::Vector4f(100, 10, 10, 1) };

public:
  void filter_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void segment_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_plane,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& objects);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud,
                                                                  float cluster_tolerance, int min_size, int max_size);
  BBox ConstructBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, size_t& obstacle_id_count);
  BBox ConstructBoundingBox_PCA(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, size_t& obstacle_id_count);
  std::vector<BBox> GetBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters,
                                     size_t& obstacle_id_count);
  std::vector<BBox> getDetections(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& segmented_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_plane,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacles_cloud, size_t& obstacle_id_count);
};

#endif  // LIDAR_OBJECT_DETECTOR_H
