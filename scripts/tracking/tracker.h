#ifndef TRACKER_H
#define TRACKER_H

#include "track.h"

class Tracker
{
public:
  void run(std::vector<BBox>& detections, std::vector<BBox>& prev_boxes, double dt);

private:
  std::vector<Track> tracks_;
};

void Tracker::run(std::vector<BBox>& current_detections, std::vector<BBox>& prev_boxes, double dt)
{

   if (current_detections.empty() || prev_boxes.empty()) {
    return;
  } 
  // Prediction step
  for (auto& track : tracks_)
  {
    track.predict(dt);
  }
  // association

  std::vector<BBox> matched_detections;
  // vector of unassociated detections
  std::vector<BBox> unmatched_detections;

  std::vector<int> pre_ids;
  std::vector<int> cur_ids;

  std::vector<int> matches;

  for (auto track : tracks_)
  {
    prev_boxes.push_back(track.getBBox());
  }
  if (!current_detections.empty())
  {
    auto connection_pairs = associateBoxes(prev_boxes, current_detections, 1.0, 1.0);
    std::cout << " in associate";

        if (connection_pairs.empty()) return;

    // Construct the connection matrix for Hungarian Algorithm's use
    auto connection_matrix =
        connectionMatrix(connection_pairs, &pre_ids, &cur_ids);

    // Use Hungarian Algorithm to solve for max-matching
    matches = hungarian(connection_matrix);

        for (int j = 0; j < matches.size(); ++j) {
      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);

      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j];  // right and matches has the same size
      const auto cur_index = searchBoxIndex(current_detections, cur_id);

      if (pre_index > -1 && cur_index > -1) {
        // change the id of the current box to the same as the previous box
        (current_detections)[cur_index].id = prev_boxes[pre_index].id;
      }
    }
  }
}

#endif
