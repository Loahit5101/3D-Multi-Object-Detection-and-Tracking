#ifndef TRACKER_H
#define TRACKER_H

#include "track.h"

class Tracker
{
public:
  void run(std::vector<BBox>& detections, double dt);
  std::vector<Track> getTracks();
  std::vector<BBox> getDetectionsFromTracks();

private:
  std::vector<Track> tracks_;
};

void Tracker::run(std::vector<BBox>& current_detections, double dt)
{
  if (current_detections.empty())
  {
    return;
  }

  if (tracks_.empty())
  {
    for (auto& detection : current_detections)
    {
      Track new_track;
      new_track.init(detection);
      tracks_.push_back(new_track);
    }
    return;
  }

  // Prediction step
  for (auto& track : tracks_)
  {
    track.predict(dt);
  }

  // Association
  std::vector<BBox> prev_boxes;
  std::vector<int> pre_ids;
  std::vector<int> cur_ids;
  std::vector<int> matches;
  std::vector<BBox> unmatched_detections;

  for (auto& track : tracks_)
  {
    prev_boxes.push_back(track.getBBox());
  }

  if (!current_detections.empty())
  {
    auto connection_pairs = associateBoxes(prev_boxes, current_detections, 1.0, 1.0);

    if (connection_pairs.empty())
    {
      // All detections are unmatched
      unmatched_detections = current_detections;
      return;
    }

    // Construct the connection matrix for Hungarian Algorithm's use
    auto connection_matrix = connectionMatrix(connection_pairs, &pre_ids, &cur_ids);

    // Use Hungarian Algorithm to solve for max-matching
    matches = hungarian(connection_matrix);

    for (int j = 0; j < matches.size(); ++j)
    {
      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);

      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j];  // right and matches have the same size
      const auto cur_index = searchBoxIndex(current_detections, cur_id);

      if (pre_index > -1 && cur_index > -1)
      {
        // change the id of the current box to the same as the previous box
        current_detections[cur_index].id = prev_boxes[pre_index].id;

        // perform update step using updated current_detections
          // Perform Kalman Filter update step
        // Extract the matched previous box and current detection
        const auto prev_bbox = prev_boxes[pre_index];
        const auto& cur_detection = current_detections[cur_index];

        // Find the corresponding track
        auto& track = tracks_[prev_bbox.id]; // Assuming track IDs match box IDs

        // Get measurement from the current detection
        BBox measurement = cur_detection;

        // Perform Kalman Filter update step
        track.update(measurement);
      }
    }

    // Extract unmatched detections
    unmatched_detections.clear();
    for (int i = 0; i < current_detections.size(); ++i)
    {
      bool matched = false;
      for (int j = 0; j < matches.size(); ++j)
      {
        if (cur_ids[j] == current_detections[i].id)
        {
          matched = true;
          break;
        }
      }
      if (!matched)
      {
        unmatched_detections.push_back(current_detections[i]);
      }
    }

    for (const auto& det : unmatched_detections)
    {
      Track tracker;
      tracker.init(det);
      tracks_.push_back(tracker);
    }
  }
}

std::vector<Track> Tracker::getTracks()
{
  return tracks_;
}

std::vector<BBox> Tracker::getDetectionsFromTracks()
{
  
  std::vector<BBox> detections;

  for(auto& track:tracks_){
 
     detections.push_back(track.getBBox());

  }
  return detections;

}
#endif
