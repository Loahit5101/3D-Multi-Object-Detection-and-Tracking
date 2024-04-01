#ifndef TRACK_H
#define TRACK_H

#include <opencv2/core.hpp>
#include "kalman_filter.h"
#include "association.h"
#include "../lidar_detection/lidar_detector.h"

class Track {
public:
    Track();

    void init(const BBox&);
    void predict(double dt);
    void update(const BBox&);

private:
    ExtendedKalmanFilter ekf_; // Kalman filter for tracking
    BBox boundingbox_;
};

Track::Track()
    :ekf_(),boundingbox_() {} 

void Track::init(const BBox& bbox) {
    boundingbox_ = bbox;
    ekf_.init_x(boundingbox_.position);
}

void Track::predict(double dt) {

    ekf_.predict(dt);
}

void Track::update(const BBox& bbox) {
   
}

#endif
