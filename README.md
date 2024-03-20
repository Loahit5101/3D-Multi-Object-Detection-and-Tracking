# Perception: Multi-Object Detection and Tracking

1. Multi-Object Detection and Tracking for Autonomous Driving / Robot Perception
2. LiDAR 3D object detection using **PCL**
3. Camera 2D object detection using YOLOv3 model
4. Tracking and sensor Fusion - ongoing
5. **Docker** and GitHub actions are used for **CI/CD**
6. GTest is used for executing tests.
7. ROS is only used for visualization

   
     ![lidar_camera_detection3-2024-03-12_19 22 58-ezgif com-speed](https://github.com/Loahit5101/3D-Multi-Object-Detection-and-Tracking/assets/55102632/d732b39b-e291-4833-b68a-1c5815f2e164)


## Dependencies
- Eigen3
- OpenCV
- PCL 1.8
- ROS Noetic
- GTest
- Boost (with components: filesystem, thread, system, program_options)
- Download KITTI tracking dataset, convert.bin pointcloud files to .pcd file
  
## Usage
 
### Detection and Tracking 
```
catkin_make
source devel/setup.bash
rosrun motl main_node DATA_PATH
```

### Testing
```
rosrun motl test_main_node 
```
### Docker

```
docker build -t mot .
docker run --network="host" -v DATA_PATH:/dataset -it mot
rosrun motl main_node /dataset
```

run roscore inside container first and replace dataset valume path in the dockerfile.

## LiDAR Object Detection:

Uses classical Poincloud Processing algorithms in PCL to detect 3D obstacles.

1. Filtering - Downsampling, Outlier removal, ROI cropping
2. Segmentation of ground plane
3. Clustering of obstacles
4. Bounding box detection around obstacles
5. Visualization of detected obstacles and ground plane

## Camera Object Detection:

Uses YOLOv3 object detection model to detect 2D obstacles.

1. Image Preprocessing
2. YOLO Object Detection
3. Non-max suppression
4. Visualization of detected obstacles

track fusion - In progress


