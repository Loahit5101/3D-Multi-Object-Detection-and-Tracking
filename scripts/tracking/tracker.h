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
  if (tracks_.empty())
  {
    tracks_.resize(detections.size());
    for (int i = 0; i < detections.size(); i++)
    {
      tracks_[i].init(detections[i]);  // Initialize track with detected bounding box
    }
  }

  else
  {
    for (auto& track : tracks_)
    {
      track.predict(dt);
    }
    // association
  }
}

#endif
