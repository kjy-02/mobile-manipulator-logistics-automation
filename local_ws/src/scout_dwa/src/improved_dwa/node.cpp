// Scout adaptation of AMSL commit 6387a9660c2064fa0545ac81d55f4bbe7962651f.
// Copyright 2020 amsl

#include "improved_dwa_planner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dwa_planner");
  ScoutDWAPlanner planner;
  planner.process();
  return 0;
}
