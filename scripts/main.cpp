#include "camera_detection/camera_detector.h"
#include "lidar_detection/lidar_detector.h"
#include "tracking/tracker.h"
#include <algorithm>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

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

std::vector<fs::directory_entry> createFileList(const std::string& input_folder_path)
{
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

visualization_msgs::MarkerArray createBoundingBoxMarkers(const std::vector<BBox>& bboxes)
{
  visualization_msgs::MarkerArray markers;

  for (size_t i = 0; i < bboxes.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";  // Set the frame ID
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_box";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = bboxes[i].position.x();
    marker.pose.position.y = bboxes[i].position.y();
    marker.pose.position.z = bboxes[i].position.z();
    marker.pose.orientation.x = bboxes[i].quaternion.x();
    marker.pose.orientation.y = bboxes[i].quaternion.y();
    marker.pose.orientation.z = bboxes[i].quaternion.z();
    marker.pose.orientation.w = bboxes[i].quaternion.w();
    marker.scale.x = bboxes[i].dimension.x();
    marker.scale.y = bboxes[i].dimension.y();
    marker.scale.z = bboxes[i].dimension.z();
    marker.color.r = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    markers.markers.push_back(marker);

    // Create text marker for ID
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "text";
    text_marker.id = i;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = bboxes[i].position.x();
    text_marker.pose.position.y = bboxes[i].position.y();
    text_marker.pose.position.z =
        bboxes[i].position.z() + bboxes[i].dimension.z() + 0.1;  // Adjust height above bounding box
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.5;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = std::to_string(bboxes[i].id);
    markers.markers.push_back(text_marker);
  }
  return markers;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_publisher");
  ros::NodeHandle nh;

  // Define ROS publishers
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);
  ros::Publisher pub_ground = nh.advertise<sensor_msgs::PointCloud2>("ground_cloud_topic", 1);
  ros::Publisher pub_object = nh.advertise<sensor_msgs::PointCloud2>("object_cloud_topic", 1);
  ros::Publisher pub_box_marker = nh.advertise<visualization_msgs::MarkerArray>("bounding_box", 1);
  ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("image_topic", 1);

  ros::Rate loop_rate(10);

  if (argc != 2)
  {
    ROS_ERROR("Usage: %s <base_path>", argv[0]);
    return 1;
  }

  std::string base_path = argv[1];

  // Construct paths using the provided base path
  std::string input_folder_path = base_path + "pcd_files/";
  std::string image_folder_path = base_path + "image_2/";
  std::string model_config = base_path + "yolov3.cfg";
  std::string model_weights = base_path + "yolov3.weights";
  std::string class_names = base_path + "coco.names";

  // Create a list of files in the input folder
  std::vector<fs::directory_entry> file_list;
  file_list = createFileList(input_folder_path);

  // Initialize object detectors
  LidarObjectDetector lidar_detector;
  CameraObjectDetector camera_detector(model_weights, model_config, class_names, 0.5, 0.4);

  Tracker multi_object_tracker;

  std::vector<cv::Mat> detection_info;

  size_t obstacle_id_count = 0;

  int frame_counter = 0;
  // Loop through point clouds and images
  std::vector<BBox> prev_lidar_boxes;
  for (const auto& entry : file_list)
  {
    // Load point cloud and image
    pcl::PointCloud<pcl::PointXYZI>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::string lidar_file_path = entry.path().string();
    std::string image_file_path = image_folder_path + entry.path().stem().string() + ".png";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadPointCloud(lidar_file_path);
    cv::Mat image = loadImage(image_file_path);

    // Detect objects in the image
    camera_detector.detectObject(image, detection_info);
    camera_detector.drawDetections(image, detection_info);

    // Get bounding boxes from LiDAR
    auto lidar_bounding_boxes =
        lidar_detector.getDetections(cloud, segmented_cloud, ground_plane, obstacles_cloud, obstacle_id_count);

    double dt = 0.0;
    multi_object_tracker.run(lidar_bounding_boxes, dt);

    auto tracked_boxes = multi_object_tracker.getDetectionsFromTracks();

    // Create visualization markers for bounding boxes
    visualization_msgs::MarkerArray bbox_markers = createBoundingBoxMarkers(tracked_boxes);

    // Publish data
    publishPointCloud(pub, cloud);
    publishPointCloud(pub_ground, ground_plane);
    publishPointCloud(pub_object, obstacles_cloud);
    pub_box_marker.publish(bbox_markers);
    publishImage(pub_image, image);

    frame_counter += 1;
    prev_lidar_boxes.clear();
    prev_lidar_boxes = std::move(lidar_bounding_boxes);

    // Spin and sleep
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
