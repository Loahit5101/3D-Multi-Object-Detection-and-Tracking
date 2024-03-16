#include <gtest/gtest.h>
#include "../scripts/lidar_detection/lidar_detector.h"

class LidarObjectDetectorTest : public ::testing::Test {
protected:
    LidarObjectDetector detector;
};

// Test filtering of point cloud
TEST_F(LidarObjectDetectorTest, FilterCloudTest) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (int i = 0; i < 100; ++i) {
        pcl::PointXYZI point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i);
        point.z = static_cast<float>(i);
        cloud->push_back(point);
    }

    detector.filter_cloud(cloud);

    EXPECT_LT(cloud->size(), 100);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
