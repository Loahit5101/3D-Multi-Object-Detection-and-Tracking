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
  size_t getId() const;
  int getConsecutiveFramesLost() const;
  void addToHistory(const Eigen::VectorXd& position);

  std::vector<Eigen::VectorXd> position_history_;

private:
  ExtendedKalmanFilter ekf_;  // Kalman filter for tracking
  BBox boundingbox_;
  int consecutive_frames_lost_;
};

Track::Track() : ekf_(), boundingbox_()
{
}

void Track::init(const BBox& bbox)
{
  boundingbox_ = bbox;
  ekf_.init_x(boundingbox_.position);
  consecutive_frames_lost_ = 0;
}

void Track::predict(double dt)
{
  consecutive_frames_lost_++;
  ekf_.predict(dt);
}

void Track::update(const BBox& bbox)
{
  consecutive_frames_lost_ = 0;
  // Extract measurement from the bounding box
  Eigen::MatrixXd measurement(3, 1);
  measurement << bbox.position.x(), bbox.position.y(), bbox.position.z();

  // Update the Kalman filter with the measurement
  ekf_.update(measurement, "LiDAR");

  Eigen::VectorXd updated_state = ekf_.getState();
  // Update the bounding box
  boundingbox_ = bbox;

  Eigen::VectorXd position(3);
  position << bbox.position.x(), bbox.position.y(), bbox.position.z();
  addToHistory(position);

  // boundingbox_.position.x() = updated_state(0);
  // boundingbox_.position.y() = updated_state(1);
  // boundingbox_.position.z() = updated_state(2);
}

size_t Track::getId() const
{
  return boundingbox_.id;
}

const BBox& Track::getBBox() const
{
  return boundingbox_;
}

int Track::getConsecutiveFramesLost() const
{
  return consecutive_frames_lost_;
}

void Track::addToHistory(const Eigen::VectorXd& position)
{
  position_history_.push_back(position);
}

#endif
