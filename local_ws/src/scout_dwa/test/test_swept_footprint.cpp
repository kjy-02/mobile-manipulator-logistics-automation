#include "../src/swept_footprint.h"
#include <cassert>
#include <iostream>
int main() {
  std::vector<geometry_msgs::Point32> footprint(4);
  const double xy[4][2]={{.4625,.3498},{.4625,-.3498},{-.4625,-.3498},{-.4625,.3498}};
  for(int i=0;i<4;++i){footprint[i].x=xy[i][0];footprint[i].y=xy[i][1];}
  const scout_collision::Bounds bounds(footprint,.02);
  const double cell=.05/std::sqrt(2.0);
  auto sweep=[&](double x,double y,double v,double w,double t){
    geometry_msgs::Pose p;p.position.x=x;p.position.y=y;
    return scout_collision::sweptClearance({p},bounds,v,w,t,cell);
  };
  // Side point is inside the old radius, outside a safe straight footprint.
  assert(sweep(0,.50,.24,0,.54)>0);
  // Left-front obstacle blocks turning toward it; turning away remains free.
  assert(sweep(.45,.45,.24,.15,3)<=0);
  assert(sweep(.45,.45,.24,-.15,3)>0);
  // A cell between coarse prediction poses must still block forward travel.
  assert(sweep(.53,0,1.,0,.3)<=0);
  // Free at rest, but the front-left corner sweeps into the obstacle.
  assert(sweep(.47,.42,0,0,3)>0);
  assert(sweep(.47,.42,0,.15,3)<=0);
  std::cout<<"6 footprint sweep scenarios passed\n";
}
