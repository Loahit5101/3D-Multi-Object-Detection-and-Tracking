#include "lidar_detector.h"


void LidarObjectDetector::segment_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_plane,  pcl::PointCloud<pcl::PointXYZI>::Ptr& objects) {

    float _max_distance = 0.1;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
   //seg.setMaxIterations(30);
    seg.setDistanceThreshold(_max_distance);
    
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false); // Extract the inliers
    extract.filter(*ground_plane); // Extract the inliers to a different point cloud

    // Optionally, you can extract the outliers to a different point cloud
    extract.setNegative(true); // Extract the outliers
    extract.filter(*objects); // Extract the outliers to a different point cloud



}

void LidarObjectDetector::cluster_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacles_cloud, float cluster_tolerance, int min_size, int max_size){

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.6); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (5000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

 for (auto& getIndices : cluster_indices)
  {  
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);

    for (auto& index : getIndices.indices)
      cluster->points.push_back(cloud->points[index]);

    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    clusters.push_back(cluster);
  }
        for(const auto& cluster : clusters)
        {
          *obstacles_cloud += *cluster;
        }


}