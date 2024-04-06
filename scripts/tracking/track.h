#ifndef TRACK_H
#define TRACK_H

#include "../lidar_detection/lidar_detector.h"
#include "association.h"
#include "kalman_filter.h"
#include <opencv2/core.hpp>

class Track
{
public:
  Track();

  void init(const BBox&);
  void predict(double dt);
  void update(const BBox&);
  const BBox& getBBox() const;

private:
  ExtendedKalmanFilter ekf_;  // Kalman filter for tracking
  BBox boundingbox_;
};

Track::Track() : ekf_(), boundingbox_()
{
}

void Track::init(const BBox& bbox)
{
  boundingbox_ = bbox;
  ekf_.init_x(boundingbox_.position);
}

void Track::predict(double dt)
{
  ekf_.predict(dt);
}

void Track::update(const BBox& bbox)
{
  // Extract measurement from the bounding box
  Eigen::MatrixXd measurement(3, 1);
  measurement << bbox.position.x(), bbox.position.y(), bbox.position.z();

  // Update the Kalman filter with the measurement
  ekf_.update(measurement, "LiDAR");

  Eigen::VectorXd updated_state = ekf_.getState();
  // Update the bounding box
  boundingbox_ = bbox;

  boundingbox_.position.x() = updated_state(0);
  boundingbox_.position.y() = updated_state(1);
  boundingbox_.position.z() = updated_state(2);
}


const BBox& Track::getBBox() const
{
  return boundingbox_;
}

#endif
