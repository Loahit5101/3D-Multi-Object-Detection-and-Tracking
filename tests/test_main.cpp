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

// Test construction of bounding boxes
TEST_F(LidarObjectDetectorTest, ConstructBoundingBoxTest) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);

    // Create a sample cluster of points
    for (int i = 0; i < 50; ++i) {
        pcl::PointXYZI point;
        point.x = static_cast<float>(i);
        point.y = static_cast<float>(i);
        point.z = static_cast<float>(i);
        cluster->push_back(point);
    }

    size_t obstacle_id_count = 0;
    BBox bbox = detector.ConstructBoundingBox(cluster, obstacle_id_count);

    // Expectation: The constructed bounding box should have a valid position and dimension
    EXPECT_TRUE(bbox.position.norm() > 0);
    EXPECT_TRUE(bbox.dimension.norm() > 0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
