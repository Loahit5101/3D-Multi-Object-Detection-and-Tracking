# Perception: Multi-Object Detection and Tracking

1. Multi-Object Detection and Tracking for Autonomous Driving / Robot Perception
2. LiDAR 3D object detection using PCL
3. Camera sensor Fusion and tracking - ongoing
4. ROS is only used for visualization

   

    ![lidar_detection-2024-03-10_13 09 18-ezgif com-video-to-gif-converter](https://github.com/Loahit5101/Perception_stack/assets/55102632/a806db8d-6cc3-490d-9ab9-c3714cdb8543)


## Dependencies
- Eigen3
- OpenCV
- PCL 1.8
- ROS 
- Boost (with components: filesystem, thread, system, program_options)
- Download KITTI tracking dataset, convert.bin pointcloud files to .pcd file
  
## Usage
```
catkin_make
source devel/setup.bash
rosrun motl main_node 
```
## LiDAR Object Detection:

Uses classical Poincloud Processing algorithms in PCL to detect 3D obstacles.

1. Filtering - Downsampling, Outlier removal, ROI cropping
2. Segmentation of ground plane
3. Clustering of obstacles
4. Bounding box detection around obstacles
5. Visualization of detected obstacles and ground plane

Camera Object Detection and track fusion - In progress


