#ifndef TRACKER_H
#define TRACKER_H

#include "track.h"

class Tracker {
public:
    void detectionstoTracks(std::vector<BBox> detections);
  
private:
    std::vector<Track> tracks_;
};

void Tracker::detectionstoTracks(std::vector<BBox> detections) {
    tracks_.resize(detections.size());

    for(int i = 0; i < detections.size(); i++) {
        tracks_[i].init(detections[i]); // Initialize track with detected bounding box
    }
}

#endif
