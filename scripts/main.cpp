#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include "lidar_detection/lidar_detector.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "camera_detection/camera_detector.h"

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

cv::Mat loadImage(const std::string& file_path)
{
    cv::Mat image = cv::imread(file_path, cv::IMREAD_COLOR);
    if (image.empty())
    {
        ROS_ERROR("Could not read image: %s", file_path.c_str());
    }
    return image;
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

void publishImage(ros::Publisher& pub, const cv::Mat& image)
{
    if (!image.empty())
    {
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(img_msg);
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
    ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("image_topic", 1);


    ros::Rate loop_rate(10);

    std::string input_folder_path = "/home/loahit/Downloads/projects/perception_project/0020/pcd_files/";
    std::string image_folder_path = "/home/loahit/Downloads/projects/perception_project/0020/image_2/";

    std::string model_config = "/home/loahit/Downloads/projects/perception_project/udacity_model/yolov3.cfg";
    std::string model_weights = "/home/loahit/Downloads/projects/perception_project/udacity_model/yolov3.weights";
    std::string class_names = "/home/loahit/Downloads/projects/perception_project/coco.names";

    std::vector<fs::directory_entry> file_list;
    file_list = createFileList(input_folder_path);

    LidarObjectDetector lidar_detector;
    CameraObjectDetector camera_detector(model_weights, model_config, class_names, 0.5, 0.4);

    std::vector<cv::Mat> detection_info;

    

    // Loop through poinclouds and images
    for (const auto& entry : file_list)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        std::string lidar_file_path = entry.path().string();
        std::string image_file_path = image_folder_path + entry.path().stem().string() + ".png";

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadPointCloud(lidar_file_path);
        cv::Mat image = loadImage(image_file_path);

        camera_detector.detectObject(image, detection_info);
        camera_detector.drawDetections(image, detection_info);

        std::cout<<"detection_info size"<< detection_info.size()<<std::endl;
  
        auto lidar_bounding_boxes = lidar_detector.get_detections(cloud, segmented_cloud, ground_plane, obstacles_cloud);

        visualization_msgs::MarkerArray bbox_markers = createBoundingBoxMarkers(lidar_bounding_boxes);

        publishPointCloud(pub, cloud);
        publishPointCloud(pub_ground, ground_plane);
        publishPointCloud(pub_object, obstacles_cloud);
        pub_box_marker.publish(bbox_markers);
        publishImage(pub_image, image);
       
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
