#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <geometry_msgs/Point32.h>

inline bool validateFootprintPolygon(const std::vector<geometry_msgs::Point32>& p, double& radius)
{
  if (p.size() < 3 || p.size() > 1000) return false;
  radius = 0;
  double area = 0;
  bool inside = false;
  auto cross = [](const geometry_msgs::Point32& a, const geometry_msgs::Point32& b,
                  const geometry_msgs::Point32& c) {
    return (double(b.x)-a.x)*(double(c.y)-a.y)-(double(b.y)-a.y)*(double(c.x)-a.x);
  };
  auto on = [&](const geometry_msgs::Point32& a, const geometry_msgs::Point32& b,
                const geometry_msgs::Point32& c) {
    return std::fabs(cross(a,b,c)) < 1e-9 &&
           c.x >= std::min(a.x,b.x) && c.x <= std::max(a.x,b.x) &&
           c.y >= std::min(a.y,b.y) && c.y <= std::max(a.y,b.y);
  };
  for (size_t i=0; i<p.size(); ++i)
  {
    const auto& a=p[i]; const auto& b=p[(i+1)%p.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
        std::fabs(a.z)>1e-6 || std::hypot(a.x-b.x,a.y-b.y)<1e-6) return false;
    radius=std::max(radius,std::hypot(double(a.x),double(a.y)));
    area += double(a.x)*b.y-double(b.x)*a.y;
    if ((a.y>0)!=(b.y>0) && 0 < (double(b.x)-a.x)*(-a.y)/(double(b.y)-a.y)+a.x)
      inside = !inside;
    for (size_t j=i+1; j<p.size(); ++j)
    {
      if (j==i+1 || (i==0 && j==p.size()-1)) continue;
      const auto& c=p[j]; const auto& d=p[(j+1)%p.size()];
      if ((cross(a,b,c)*cross(a,b,d)<0 && cross(c,d,a)*cross(c,d,b)<0) ||
          on(a,b,c) || on(a,b,d) || on(c,d,a) || on(c,d,b)) return false;
    }
  }
  return inside && std::fabs(area)>1e-6 && std::isfinite(radius);
}
