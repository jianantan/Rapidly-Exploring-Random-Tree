#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_

#include <random>
#include <iostream>

#include <string>
#include <vector>
#include <cmath>
#include <math.h>  
#include <chrono>
#include <thread>
#include <unistd.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>

namespace rrt_planner
{

class Point2D
{
public:
  Point2D(): x_(0), y_(0) {}
  Point2D(int x, int y): x_(x), y_(y) {}

  int x() const
  {
    return x_;
  }
  int y() const
  {
    return y_;
  }
  void x(int x)
  {
    x_ = x;
  }
  void y(int y)
  {
    y_ = y;
  }

private:
  int x_;
  int y_;
};

class RRTPlanner
{
public:
  RRTPlanner();

  ~RRTPlanner() = default;

  void run();

  void plan(const int max_dist, const int search_r, const std::string variant);
  void mapCallback(const nav_msgs::OccupancyGrid::Ptr &);
  void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

private:
  ros::NodeHandlePtr node_;
  ros::NodeHandlePtr node_params_;

  ros::Subscriber map_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;

  int max_dist_;
  int search_radius_;
  std::string variant_;

  bool map_received_;
  std::unique_ptr<cv::Mat> map_;
  nav_msgs::OccupancyGrid::Ptr map_grid_;

  bool init_pose_received_;
  Point2D init_pose_;

  bool goal_received_;
  Point2D goal_;

  void publishPath(std::vector<Point2D> completePath);
  bool isPointUnoccupied(const Point2D & p);
  bool isNotObstacleFree(int start_x, int start_y, int end_x, int end_y);
  void buildMapImage();
  void displayMapImage(int delay = 1);
  void drawGoalInitPose();
  void drawCircle(Point2D & p, int radius, const cv::Scalar & color);
  void drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness = 1);
  int euclideanDist(int x1, int y1, int x2, int y2);
  inline geometry_msgs::PoseStamped pointToPose(const Point2D &);
  inline void poseToPoint(Point2D &, const geometry_msgs::Pose &);
  inline int toIndex(int, int);




  

}; // RRTPlanner

} // rrt_planner

#endif // RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_