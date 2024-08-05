#include "subzerolib/api/spline/spline.hpp"

int find_pose_index(std::vector<point_s> &vec, pose_s pose) {
  double min_d = std::numeric_limits<double>::max();
  int index = 0;
  for (int i = 0; i < vec.size(); ++i) {
    double dist = pose.dist(vec[i]);
    if (dist < min_d) {
      index = i;
      min_d = dist;
    }
  }
  double dx = pose.x - vec[index].x;
  double dy = pose.y - vec[index].y;
  if (index != vec.size() - 1) {
    double sample_dx = vec[index + 1].x - vec[index].x;
    double sample_dy = vec[index + 1].y - vec[index].y;
    bool same_dir = sample_dx * dx + sample_dy * dy > 0;
    if (same_dir)
      index += 1;
  }
  return index;
}

std::vector<pose_s> interpolate_heading(std::vector<point_s> path,
                                        std::vector<pose_s> ctrl_points) {
  // create a blank heading path
  std::vector<pose_s> generated(path.size());
  transform(path.begin(),
            path.end(),
            generated.begin(),
            [](point_s point) -> pose_s { return pose_s{point, 0.0}; });

  // find control point indices
  std::vector<int> ctrl_indices;
  for (auto &c : ctrl_points) {
    int i = find_pose_index(path, c);
    generated[i].x = c.x;
    generated[i].y = c.y;
    generated[i].h = c.h;
    ctrl_indices.push_back(i);
  }

  std::vector<double> distances(generated.size());
  distances[0] = 0;
  // generate distances
  for (int i = 1; i < distances.size(); ++i) {
    distances[i] = distances[i - 1] + generated[i - 1].dist(generated[i]);
  }

  // lerp headings
  for (int i = 0; i < generated.size(); ++i) {
    // only lerp for non-control points
    if (std::find(ctrl_indices.begin(), ctrl_indices.end(), i) ==
        ctrl_indices.end()) {
      // find the indices of surrounding control points
      int start_index = 0;
      int end_index = 0;
      for (int j = 1; j < ctrl_indices.size(); ++j) {
        if (ctrl_indices[j] > i) {
          start_index = ctrl_indices[j - 1];
          end_index = ctrl_indices[j];
          break;
        }
      }
      double factor = (distances[i] - distances[start_index]) /
                      (distances[end_index] - distances[start_index]);
      double h0 = generated[start_index].h;
      double h1 = generated[end_index].h;
      generated[i].h = h0 + factor * shorter_turn(h0, h1, 360.0);
    }
  }

  return generated;
}
