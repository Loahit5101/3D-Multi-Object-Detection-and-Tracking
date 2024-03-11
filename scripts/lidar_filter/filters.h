#ifndef FILTERS_H
#define FILTERS_H

#include "base_filter.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>


class StatisticalOutlierRemoval: public BaseFilter{

private:

   int meanK_;
   double stddevMulThresh_;

public:

  StatisticalOutlierRemoval(int meanK, double stddevMulThresh)
        : meanK_(meanK), stddevMulThresh_(stddevMulThresh) {}

  void apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) override{

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK_);
    sor.setStddevMulThresh (stddevMulThresh_);
    sor.filter (*cloud);
  }

};

class DownsampleFilter: public BaseFilter{

private:
  
  float LeafSize_;

public:

  DownsampleFilter(float LeafSize): LeafSize_(LeafSize){}

  void apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) override{

     pcl::VoxelGrid<pcl::PointXYZI> vgf;
     vgf.setInputCloud (cloud);
     vgf.setLeafSize (LeafSize_, LeafSize_, LeafSize_);
     vgf.filter (*cloud);

  }

};

class CropBoxFilter: public BaseFilter{

private:
  
  Eigen::Vector4f minPoint_;
  Eigen::Vector4f maxPoint_;

public:

  CropBoxFilter(const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint): minPoint_(minPoint), maxPoint_(maxPoint){}

  void apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) override{

    pcl::CropBox<pcl::PointXYZI> region(true);
    region.setMin(minPoint_);
    region.setMax(maxPoint_);
    region.setInputCloud(cloud);
    region.filter(*cloud);

  }

};

#endif


