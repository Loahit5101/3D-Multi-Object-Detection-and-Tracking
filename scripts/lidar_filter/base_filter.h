#ifndef BASE_FILTER_H
#define BASE_FILTER_H

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>

class BaseFilter{

 public:
  BaseFilter() = default;
 
  virtual ~BaseFilter() =default;
 
  virtual void apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) = 0;

};

#endif // BASE_FILTER_H


