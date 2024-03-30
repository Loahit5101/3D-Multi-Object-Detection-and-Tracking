#ifndef ASSOCIATION_H
#define ASSOCIATION_H

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "../lidar_detection/lidar_detector.h"

bool compareBoxes(const BBox &a, const BBox &b,
                                    const float displacement_thresh,
                                    const float iou_thresh) {

  const float dis =
      sqrt((a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) +
           (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) +
           (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

  const float a_max_dim =
      std::max(a.dimension[0], std::max(a.dimension[1], a.dimension[2]));
  const float b_max_dim =
      std::max(b.dimension[0], std::max(b.dimension[1], b.dimension[2]));
  const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

  const float x_dim =
      2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]);
  const float y_dim =
      2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]);
  const float z_dim =
      2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]);

  if (ctr_dis <= displacement_thresh && x_dim <= iou_thresh &&
      y_dim <= iou_thresh && z_dim <= iou_thresh) {
    return true;
  } else {
    return false;
  }
}

std::vector<std::vector<int>> associateBoxes(
    const std::vector<BBox> &prev_boxes, const std::vector<BBox> &curr_boxes,
    const float displacement_thresh, const float iou_thresh) {
  std::vector<std::vector<int>> connection_pairs;

  for (auto &prev_box : prev_boxes) {
    for (auto &curBox : curr_boxes) {
      // Add the indices of a pair of similar boxes to the matrix
      if (compareBoxes(curBox, prev_box, displacement_thresh, iou_thresh)) {
        connection_pairs.push_back({prev_box.id, curBox.id});
      }
    }
  }

  return connection_pairs;
}

std::vector<std::vector<int>> connectionMatrix(
    const std::vector<std::vector<int>> &connection_pairs,
    std::vector<int> *left, std::vector<int> *right) {

  for (auto &pair : connection_pairs) {
    const bool left_found = std::any_of(left->begin(), left->end(),
                                        [pair](int i) { return i == pair[0]; });
    if (!left_found) left->push_back(pair[0]);
    const bool right_found = std::any_of(
        right->begin(), right->end(), [pair](int j) { return j == pair[1]; });
    if (!right_found) right->push_back(pair[1]);
  }

  std::vector<std::vector<int>> connection_matrix(
      left->size(), std::vector<int>(right->size(), 0));

  for (auto &pair : connection_pairs) {
    int left_index = -1;
    for (int i = 0; i < left->size(); ++i) {
      if ((*left)[i] == pair[0]) left_index = i;
    }

    int right_index = -1;
    for (int i = 0; i < right->size(); ++i) {
      if ((*right)[i] == pair[1]) right_index = i;
    }

    if (left_index != -1 && right_index != -1)
      connection_matrix[left_index][right_index] = 1;
  }

  return connection_matrix;
}

bool hungarianFind(
    const int i, const std::vector<std::vector<int>> &connection_matrix,
    std::vector<bool> *right_connected, std::vector<int> *right_pair) {
  for (int j = 0; j < connection_matrix[0].size(); ++j) {
    if (connection_matrix[i][j] == 1 && (*right_connected)[j] == false) {
      (*right_connected)[j] = true;

      if ((*right_pair)[j] == -1 ||
          hungarianFind((*right_pair)[j], connection_matrix, right_connected,
                        right_pair)) {
        (*right_pair)[j] = i;
        return true;
      }
    }
  }

  return false;
}

std::vector<int> hungarian(
    const std::vector<std::vector<int>> &connection_matrix) {
  std::vector<bool> right_connected(connection_matrix[0].size(), false);
  std::vector<int> right_pair(connection_matrix[0].size(), -1);

  int count = 0;
  for (int i = 0; i < connection_matrix.size(); ++i) {
    if (hungarianFind(i, connection_matrix, &right_connected, &right_pair))
      count++;
  }

  return right_pair;
}

int searchBoxIndex(const std::vector<BBox> &boxes,
                                             const int id) {
  for (int i = 0; i < boxes.size(); i++) {
    if (boxes[i].id == id) return i;
  }

  return -1;
}

#endif