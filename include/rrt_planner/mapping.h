#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_MAPPING_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_MAPPING_H_

#include <ros/ros.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <string>
#include <filesystem>
#include <fstream>
#include "yaml-cpp/yaml.h"

namespace mapping
{

class Mapping
{
  
public:
  Mapping();

  ~Mapping() = default;

  void run();

  static void mouseHandler(int event, int x, int y, int, void*);

private:
  ros::NodeHandlePtr node_;
  ros::NodeHandlePtr node_params_;

  std::string resources_dir_;
  std::string cfg_dir_;

  int width_;
  int height_;

  

  
};

} // mapping

#endif // RRT_PLANNER_INCLUDE_RRT_PLANNER_MAPPING_H_