#include "ros/ros.h"
#include "rrt_planner/rrt_planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrt_planner");
  rrt_planner::RRTPlanner rrt_planner_;
  rrt_planner_.run();
  return 0;
}
