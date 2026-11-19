#ifndef DWA_PLANNER_GOAL_OBSTACLE_FILTER_H
#define DWA_PLANNER_GOAL_OBSTACLE_FILTER_H

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace dwa_planner
{
inline bool obstacle_is_near_goal(double x, double y, double goal_x, double goal_y, double radius)
{
  return std::isfinite(radius) && radius > 0.0 && std::isfinite(goal_x) && std::isfinite(goal_y) &&
         std::hypot(x - goal_x, y - goal_y) <= radius;
}

// The obstacle positions and goal must be expressed in the same frame.
// Filter a planning copy; keep the original sensor/map obstacles intact.
template <typename PoseSequence>
void filter_obstacles_near_goal(PoseSequence &poses, double goal_x, double goal_y, double radius)
{
  if (!std::isfinite(radius) || radius <= 0.0 || !std::isfinite(goal_x) || !std::isfinite(goal_y))
    return;

  poses.erase(std::remove_if(poses.begin(), poses.end(), [=](const auto &pose) {
    return obstacle_is_near_goal(pose.position.x, pose.position.y, goal_x, goal_y, radius);
  }), poses.end());
}

// Match the planner's existing ray sampling of an axis-aligned local map.
// Skip ignored hits BEFORE terminating the ray, so obstacles behind them remain visible.
template <typename Grid, typename PoseSequence>
void extract_map_obstacles(const Grid &map, PoseSequence &poses, double angle_resolution,
                           double goal_x = 0.0, double goal_y = 0.0, double ignore_radius = 0.0)
{
  poses.clear();
  if (!std::isfinite(map.info.resolution) || map.info.resolution <= 0.0 ||
      !std::isfinite(angle_resolution) || angle_resolution <= 0.0 ||
      map.data.size() < static_cast<std::size_t>(map.info.width) * map.info.height)
    return;

  const double max_search_dist = std::hypot(map.info.origin.position.x, map.info.origin.position.y);
  const double pi = std::acos(-1.0);
  for (float angle = -pi; angle <= pi; angle += angle_resolution)
  {
    for (float dist = 0.0; dist <= max_search_dist; dist += map.info.resolution)
    {
      typename PoseSequence::value_type pose;
      pose.position.x = dist * std::cos(angle);
      pose.position.y = dist * std::sin(angle);
      const int x = std::floor((pose.position.x - map.info.origin.position.x) / map.info.resolution);
      const int y = std::floor((pose.position.y - map.info.origin.position.y) / map.info.resolution);
      if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) || y >= static_cast<int>(map.info.height))
        continue;
      if (map.data[x + y * map.info.width] != 100)
        continue;
      if (obstacle_is_near_goal(pose.position.x, pose.position.y, goal_x, goal_y, ignore_radius))
        continue;
      poses.push_back(pose);
      break;
    }
  }
}
}  // namespace dwa_planner

#endif
