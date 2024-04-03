#ifndef TRACKER_H
#define TRACKER_H

#include "track.h"

class Tracker
{
public:
  void run(std::vector<BBox>& detections, double dt);

private:
  std::vector<Track> tracks_;
};

void Tracker::run(std::vector<BBox>& detections, double dt)
{
  for (auto& track : tracks_)
  {
    track.predict(dt);
  }
  // association

  std::vector<BBox> matched_detections;
  // vector of unassociated detections
  std::vector<BBox> unmatched_detections;

  // tracks to prev_boxes
  std::vector<BBox> prev_boxes;

  for (auto track : tracks_)
  {
    prev_boxes.push_back(track.getBBox());
  }
  if (!detections.empty())
  {
    associateBoxes(prev_boxes, detections, 0.5, 0.5);
    std::cout << " in associate";
  }
}

#endif
