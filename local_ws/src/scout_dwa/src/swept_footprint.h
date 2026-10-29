#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>

namespace scout_collision {
// Bounding rectangle is exact for the configured rectangular Scout footprint.
// An occupied grid cell is covered by its circumscribed disk. Intermediate
// poses include a motion envelope so gaps between samples cannot skip a cell.
struct Bounds {
  double min_x=0, max_x=0, min_y=0, max_y=0, radius=0;
  template<class Points> explicit Bounds(const Points& points, double padding=0) {
    min_x=min_y=std::numeric_limits<double>::infinity();
    max_x=max_y=-std::numeric_limits<double>::infinity();
    for(const auto& p:points) {
      min_x=std::min(min_x,double(p.x));max_x=std::max(max_x,double(p.x));
      min_y=std::min(min_y,double(p.y));max_y=std::max(max_y,double(p.y));
    }
    min_x-=padding;max_x+=padding;min_y-=padding;max_y+=padding;
    radius=std::hypot(std::max(std::abs(min_x),std::abs(max_x)),
                      std::max(std::abs(min_y),std::abs(max_y)));
  }
};
inline double clearance(const std::vector<geometry_msgs::Pose>& obstacles,
                        const Bounds& b, double x,double y,double yaw,
                        double obstacle_radius, double cap) {
  double result=cap;const double c=std::cos(yaw),s=std::sin(yaw);
  for(const auto& p:obstacles) {
    const double dx=p.position.x-x,dy=p.position.y-y;
    // Cheap conservative rejection before transforming a distant point.
    if(std::abs(dx)>b.radius+cap+obstacle_radius ||
       std::abs(dy)>b.radius+cap+obstacle_radius) continue;
    const double rx=c*dx+s*dy,ry=-s*dx+c*dy;
    const double qx=std::max({b.min_x-rx,0.0,rx-b.max_x});
    const double qy=std::max({b.min_y-ry,0.0,ry-b.max_y});
    result=std::min(result,std::hypot(qx,qy)-obstacle_radius);
    if(result<=0) return result;
  }
  return result;
}
inline double sweptClearance(const std::vector<geometry_msgs::Pose>& obstacles,
                             const Bounds& b,double v,double w,double horizon,
                             double cell_radius,double cap=.2) {
  const double rate=std::abs(v)+b.radius*std::abs(w);
  const int steps=std::max(1,int(std::ceil(horizon*rate/.01)));
  const double dt=horizon/steps;
  // The footprint can move at most rate*dt/2 between a pose and its nearest
  // sampled pose. Expand occupied cells by that bound for the full sweep.
  const double envelope=cell_radius+rate*dt*.5;
  double result=cap;
  for(int i=0;i<=steps;++i) {
    const double t=i*dt,a=w*t;
    const double x=std::abs(w)<1e-9?v*t:v/w*std::sin(a);
    const double y=std::abs(w)<1e-9?0:v/w*(1-std::cos(a));
    result=std::min(result,clearance(obstacles,b,x,y,a,envelope,cap));
    if(result<=0) return result;
  }
  return result;
}
} // namespace scout_collision
