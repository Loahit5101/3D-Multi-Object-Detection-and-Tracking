
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

struct BBox
{
    Eigen::Vector3f position;
    Eigen::Vector3f dimension;

    BBox(const Eigen::Vector3f& position, const Eigen::Vector3f& dimension)
        : position(position), dimension(dimension){}

};

class LidarObjectDetector{

  private:

  public:

   void segment_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_plane,  pcl::PointCloud<pcl::PointXYZI>::Ptr& objects);
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacle_cloud, float cluster_tolerance, int min_size, int max_size);   
   BBox ConstructBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster);
   std::vector<BBox> GetBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters);

};


