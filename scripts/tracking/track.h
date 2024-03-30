#ifndef TRACK_H
#define TRACK_H

#include <opencv2/core.hpp>
#include "kalman_filter.h"
#include "association.h"
#include "../lidar_detection/lidar_detector.h"

class Track {
public:
    Track(int state_dim = 6, double dt = 0.1, double process_variance = 0.01);

    void init(const BBox&);
    void predict(double dt);
    void update(const BBox&);

private:
    ExtendedKalmanFilter ekf_; // Kalman filter for tracking
    BBox boundingbox_;
};

Track::Track(int state_dim, double dt, double process_variance)
    : ekf_(state_dim, dt, process_variance), boundingbox_() {} 

void Track::init(const BBox& bbox) {
    boundingbox_ = bbox;
}

void Track::predict(double dt) {
   
}

void Track::update(const BBox& bbox) {
   
}

#endif
