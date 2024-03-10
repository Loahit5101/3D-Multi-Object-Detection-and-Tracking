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

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> LidarObjectDetector::cluster_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacles_cloud, float cluster_tolerance, int min_size, int max_size){

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

  return clusters;      


}


BBox LidarObjectDetector::ConstructBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster)
{
  pcl::PointXYZI min_pt, max_pt;
 
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  
  const Eigen::Vector3f position((max_pt.x + min_pt.x)/2, (max_pt.y + min_pt.y)/2, (max_pt.z + min_pt.z)/2);
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), (max_pt.z - min_pt.z));

  return BBox(position, dimension);

}


std::vector<BBox> LidarObjectDetector::GetBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>  clusters){

    std::vector<BBox> bounding_boxes;
    for(size_t i = 0; i < clusters.size(); ++i ){

        BBox box = ConstructBoundingBox(clusters[i]);
        bounding_boxes.push_back(box);

    }

    return bounding_boxes;
}