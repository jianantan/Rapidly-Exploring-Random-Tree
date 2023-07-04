#include "ros/ros.h"
#include "rrt_planner/mapping.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mapping");
  mapping::Mapping mapping_;
  mapping_.run();
  return 0;
}