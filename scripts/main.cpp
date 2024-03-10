#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include "lidar_filter/filters.h"
#include "lidar_detection/lidar_detector.h"

namespace fs = boost::filesystem;

bool compareFiles(const fs::directory_entry& entry1, const fs::directory_entry& entry2)
{
    return entry1.path().filename().string() < entry2.path().filename().string();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr loadPointCloud(const std::string& file_path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1)
    {
        ROS_ERROR("Could not read file: %s", file_path.c_str());
        return nullptr;
    }
    return cloud;
}

void publishPointCloud(ros::Publisher& pub, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    if (cloud)
    {
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*cloud, pcl_msg);
        pcl_msg.header.frame_id = "velodyne";
        pub.publish(pcl_msg);
    }
}

std::vector<fs::directory_entry> createFileList(const std::string& input_folder_path){

    std::vector<fs::directory_entry> file_list;
    for (const auto& entry : fs::directory_iterator(input_folder_path))
    {
        if (entry.path().extension() == ".pcd")
        {
            file_list.push_back(entry);
        }
    }

    std::sort(file_list.begin(), file_list.end(), compareFiles);

    return file_list;
}

visualization_msgs::MarkerArray createBoundingBoxMarkers(const std::vector<BBox>& bboxes) {
    visualization_msgs::MarkerArray markers;
    

    for(size_t i = 0;i<bboxes.size();i++)
    {
        visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // Set the frame ID
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_box";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = bboxes[i].position.x();
    marker.pose.position.y = bboxes[i].position.y();
    marker.pose.position.z = bboxes[i].position.z();
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = bboxes[i].dimension.x();
    marker.scale.y = bboxes[i].dimension.y();
    marker.scale.z = bboxes[i].dimension.z();
    marker.color.r = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    markers.markers.push_back(marker);
    }
    return markers;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);
    ros::Publisher pub_ground = nh.advertise<sensor_msgs::PointCloud2>("ground_cloud_topic", 1);
    ros::Publisher pub_object = nh.advertise<sensor_msgs::PointCloud2>("object_cloud_topic", 1);
    ros::Publisher pub_box_marker = nh.advertise<visualization_msgs::MarkerArray>("bounding_box", 1);

    ros::Rate loop_rate(10);

    std::string input_folder_path = "/home/loahit/Downloads/projects/perception_project/0020/pcd_files/";

    std::vector<fs::directory_entry> file_list;
    file_list = createFileList(input_folder_path);

    // Loop through poinclouds
    for (const auto& entry : file_list)
    {
        std::string input_file_path = entry.path().string();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadPointCloud(input_file_path);
        pcl::PointCloud<pcl::PointXYZI>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZI>);
  
        
        StatisticalOutlierRemoval stat(50,1.0);
        DownsampleFilter Downsample(0.1f);
        CropBoxFilter crop(Eigen::Vector4f (-100, -5, -10, 1),Eigen::Vector4f (100, 10, 10, 1));
        

        Downsample.apply_filter(cloud);
        stat.apply_filter(cloud);
        crop.apply_filter(cloud);

        LidarObjectDetector lidar_detector;
        lidar_detector.segment_plane(cloud, ground_plane, segmented_cloud);

        float cluster_tolerance=0.6;
        int min_size=100;
        int max_size = 5000;
        

        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        auto obstacles_cluster_vector = lidar_detector.cluster_cloud(segmented_cloud,obstacles_cloud, cluster_tolerance,min_size,max_size);

        auto bounding_boxes = lidar_detector.GetBoundingBoxes(obstacles_cluster_vector);

        std::cout << "Dimensions (length, width, height): " << bounding_boxes[0].dimension.transpose() << std::endl;

        visualization_msgs::MarkerArray bbox_markers = createBoundingBoxMarkers(bounding_boxes);

        publishPointCloud(pub, cloud);
        publishPointCloud(pub_ground, ground_plane);
        publishPointCloud(pub_object, obstacles_cloud);

        pub_box_marker.publish(bbox_markers);
       

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
