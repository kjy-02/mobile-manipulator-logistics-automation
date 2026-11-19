#include "dwa_planner/goal_obstacle_filter.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

struct Point
{
  double x;
  double y;
};

struct Pose
{
  Point position;
};

struct Grid
{
  struct Info
  {
    unsigned int width;
    unsigned int height;
    double resolution;
    Pose origin;
  } info;
  std::vector<int> data;
};

void require(bool condition, const char *message)
{
  if (!condition)
  {
    std::cerr << message << std::endl;
    std::exit(1);
  }
}

int main()
{
  const std::vector<Pose> original = {{{5.0, 2.0}}, {{5.75, 2.0}}, {{5.751, 2.0}}, {{0.0, 0.0}}};
  auto filtered = original;
  dwa_planner::filter_obstacles_near_goal(filtered, 5.0, 2.0, 0.75);
  require(filtered.size() == 2, "Goal center and 0.75m boundary must be removed; outside retained");
  require(filtered[0].position.x == 5.751, "Point just outside radius must remain");
  require(filtered[1].position.x == 0.0, "Mask must be goal-centered, not vehicle-centered");
  require(original.size() == 4, "Original sensor/map list must remain unchanged");

  for (double radius : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity()})
  {
    filtered = original;
    dwa_planner::filter_obstacles_near_goal(filtered, 5.0, 2.0, radius);
    require(filtered.size() == original.size(), "Disabled or invalid radius must retain obstacles");
  }

  filtered = original;
  dwa_planner::filter_obstacles_near_goal(filtered, std::numeric_limits<double>::quiet_NaN(), 2.0, 0.75);
  require(filtered.size() == original.size(), "Invalid goal must not erase obstacles");

  filtered = original;
  dwa_planner::filter_obstacles_near_goal(filtered, 0.0, 0.0, 0.75);
  require(filtered.size() == 3, "Recompute from raw observations when goal changes");
  require(filtered[0].position.x == 5.0, "Previously ignored point must return for a different goal");

  // A common rigid transform must not change which obstacle points are ignored.
  const double yaw = 0.6;
  auto transform = [=](Point point) {
    return Point{std::cos(yaw) * point.x - std::sin(yaw) * point.y - 3.0,
                 std::sin(yaw) * point.x + std::cos(yaw) * point.y + 1.0};
  };
  filtered = {{{5.2, 2.0}}, {{5.9, 2.0}}, {{0.0, 0.0}}};
  for (auto &pose : filtered)
    pose.position = transform(pose.position);
  const Point goal = transform({5.0, 2.0});
  dwa_planner::filter_obstacles_near_goal(filtered, goal.x, goal.y, 0.75);
  require(filtered.size() == 2, "Mask must remain consistent under translation and rotation");

  filtered.clear();
  dwa_planner::filter_obstacles_near_goal(filtered, 5.0, 2.0, 0.75);
  require(filtered.empty(), "Empty obstacle list must be supported");

  Grid map;
  map.info = {120, 120, 0.05, {{-3.0, -3.0}}};
  map.data.resize(120 * 120, 0);
  for (unsigned int y = 0; y < map.info.height; ++y)
  {
    for (unsigned int x = 0; x < map.info.width; ++x)
    {
      const double px = -3.0 + (x + 0.5) * 0.05;
      const double py = -3.0 + (y + 0.5) * 0.05;
      const bool table = px > 0.85 && px < 1.05 && std::abs(py) < 0.2;
      const bool wall = px > 2.0 && px < 2.15 && std::abs(py) < 0.2;
      if (table || wall)
        map.data[x + y * map.info.width] = 100;
    }
  }
  const auto map_before = map.data;
  std::vector<Pose> raw;
  dwa_planner::extract_map_obstacles(map, raw, 0.087);
  require(!raw.empty(), "Unfiltered map must detect table");

  // Regression: deleting first-hit samples used to hide the known wall behind the table.
  filtered = raw;
  dwa_planner::filter_obstacles_near_goal(filtered, 1.0, 0.0, 0.75);
  require(filtered.empty(), "Fixture must reproduce the old post-extraction filtering failure");
  dwa_planner::extract_map_obstacles(map, filtered, 0.087, 1.0, 0.0, 0.75);
  require(!filtered.empty(), "Outside wall behind ignored table must remain detected");
  for (const auto &pose : filtered)
  {
    require(pose.position.x > 1.9, "Remaining hits must belong to the outside wall");
    require(std::hypot(pose.position.x - 1.0, pose.position.y) > 0.75, "Inside hits must be skipped");
  }
  require(map.data == map_before, "Map extraction must not alter the original occupancy grid");

  for (double radius : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()})
  {
    dwa_planner::extract_map_obstacles(map, filtered, 0.087, 1.0, 0.0, radius);
    require(filtered.size() == raw.size(), "Disabled extraction mask must restore original samples");
    for (std::size_t i = 0; i < raw.size(); ++i)
      require(filtered[i].position.x == raw[i].position.x && filtered[i].position.y == raw[i].position.y,
              "Disabled extraction must preserve sample positions");
  }

  dwa_planner::extract_map_obstacles(map, filtered, 0.087, 5.0, 5.0, 0.75);
  require(filtered.size() == raw.size(), "Different goal must reveal table again");

  std::cout << "PASS: mask boundary, settings, raw data, goal changes, frames, outside wall behind ignored table"
            << std::endl;
  return 0;
}
