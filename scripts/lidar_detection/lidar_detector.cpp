#include "lidar_detector.h"

void LidarObjectDetector::filter_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  crop.apply_filter(cloud);
  outlierRemovalFilter.apply_filter(cloud);
  Downsample.apply_filter(cloud);
}

void LidarObjectDetector::segment_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_plane,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& objects)
{
  float _max_distance = 0.1;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setMaxIterations(30);
  seg.setDistanceThreshold(_max_distance);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  extract.setNegative(false);     // Extract the inliers
  extract.filter(*ground_plane);  // Extract the inliers to a different point cloud

  extract.setNegative(true);  // Extract the outliers
  extract.filter(*objects);   // Extract the outliers to a different point cloud
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> LidarObjectDetector::cluster_cloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacles_cloud,
    float cluster_tolerance, int min_size, int max_size)
{
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.6);  // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(5000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

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
  for (const auto& cluster : clusters)
  {
    *obstacles_cloud += *cluster;
  }

  return clusters;
}

BBox LidarObjectDetector::ConstructBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, size_t& obstacle_id_count)
{
  pcl::PointXYZI min_pt, max_pt;

  pcl::getMinMax3D(*cluster, min_pt, max_pt);

  const Eigen::Vector3f position((max_pt.x + min_pt.x) / 2, (max_pt.y + min_pt.y) / 2, (max_pt.z + min_pt.z) / 2);
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), (max_pt.z - min_pt.z));

  return BBox(obstacle_id_count, position, dimension);
}

BBox LidarObjectDetector::ConstructBoundingBox_PCA(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster,
                                                   size_t& obstacle_id_count)
{
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  const float box_height = max_pt.z - min_pt.z;
  const float box_z = (max_pt.z + min_pt.z) / 2;

  // get centroid
  Eigen::Vector4f pca_centroid;
  pcl::compute3DCentroid(*cluster, pca_centroid);

  // project to plane z = z_centroid
  for (size_t i = 0; i < cluster->size(); ++i)
  {
    cluster->points[i].z = pca_centroid(2);
  }

  // get principal axes & transform the original cloud to PCA coordinates
  pcl::PointCloud<pcl::PointXYZI>::Ptr pca_projected_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCA<pcl::PointXYZI> pca;
  pca.setInputCloud(cluster);
  pca.project(*cluster, *pca_projected_cloud);

  const auto eigen_vectors = pca.getEigenVectors();

  pcl::getMinMax3D(*pca_projected_cloud, min_pt, max_pt);
  const Eigen::Vector3f meanDiagonal = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // get oriented boxes
  const Eigen::Quaternionf quaternion(eigen_vectors);
  const Eigen::Vector3f position = eigen_vectors * meanDiagonal + pca_centroid.head<3>();
  const Eigen::Vector3f dimension((max_pt.x - min_pt.x), (max_pt.y - min_pt.y), box_height);

  return BBox(obstacle_id_count, position, dimension, quaternion);
}

std::vector<BBox> LidarObjectDetector::GetBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters,
                                                        size_t& obstacle_id_count)
{
  std::vector<BBox> bounding_boxes;
  for (size_t i = 0; i < clusters.size(); ++i)
  {
    BBox box = ConstructBoundingBox(clusters[i], obstacle_id_count);
    bounding_boxes.push_back(box);

    obstacle_id_count = (obstacle_id_count < SIZE_MAX) ? ++obstacle_id_count : 0;
  }

  return bounding_boxes;
}

std::vector<BBox> LidarObjectDetector::getDetections(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& segmented_cloud,
                                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_plane,
                                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& obstacles_cloud,
                                                     size_t& obstacle_id_count)
{
  filter_cloud(cloud);
  segment_plane(cloud, ground_plane, segmented_cloud);
  auto obstacles_cluster_vector = cluster_cloud(segmented_cloud, obstacles_cloud, 0.6, 100, 5000);
  auto bounding_boxes = GetBoundingBoxes(obstacles_cluster_vector, obstacle_id_count);

  return bounding_boxes;
}