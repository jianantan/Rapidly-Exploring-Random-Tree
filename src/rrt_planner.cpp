#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner()
: map_received_(false),
  init_pose_received_(false),
  goal_received_(false)
{
  // Initialize node handles
  node_.reset(new ros::NodeHandle());
  node_params_.reset(new ros::NodeHandle("~"));
}

void RRTPlanner::run()
{
  // Get map and path topics from parameter server
  std::string map_topic, path_topic;
  node_->param<std::string>("map_topic", map_topic, "/map");
  node_->param<std::string>("path_topic", path_topic, "/path");
  node_params_->param<int>("max_dist", max_dist_, 10);
  node_params_->param<int>("search_radius", search_radius_, 30);
  node_params_->param<std::string>("variant", variant_, "rrt_star");

  // Subscribe to map topic
  map_sub_ = node_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
    map_topic, 1, &RRTPlanner::mapCallback, this);
  
  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = node_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
    "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

  // Subscribe to goal topic that is published by RViz
  goal_sub_ = node_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
    "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

  // Advertise topic where calculated path is going to be published
  path_pub_ = node_->advertise<nav_msgs::Path>(path_topic, 1, true);

  ROS_INFO("Initialization complete!");
  if(variant_ == "rrt_star")
  {
    ROS_INFO("Variant: RRT*");
    ROS_INFO("Maximum distance of candidate point from proximal vertex: %d", max_dist_);
    ROS_INFO("Search radius: %d", search_radius_);
  }
  else
  {
    ROS_INFO("Variant: Vanilla-RRT");
    ROS_INFO("Maximum distance of candidate point from proximal vertex: %d", max_dist_);
  }
  
  while (ros::ok()) {
    // if map, initial pose, and goal have been received
    // build the map image, draw initial pose and goal, and plan
    if (map_received_ && init_pose_received_ && goal_received_) {
      buildMapImage();
      drawGoalInitPose();
      //********************************************************************************************************************
      // Configure max_dist, search_r, and variant here
      plan(max_dist_, search_radius_, variant_);
      //********************************************************************************************************************
    } else {
      if (map_received_) {
        displayMapImage();
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
  map_grid_ = msg;

  // Build and display the map image
  buildMapImage();
  displayMapImage();

  // Reset these values for a new planning iteration
  map_received_ = true;
  init_pose_received_ = false;
  goal_received_ = false;

  ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  if (init_pose_received_) {
    buildMapImage();
  }

  // Convert mas to Point2D
  poseToPoint(init_pose_, msg->pose.pose);
  
  // Information on position
  ROS_INFO("initial position is ");
  printf("(%d, %d)\n", init_pose_.y(), map_grid_->info.height - init_pose_.x() - 1);

  // Reject the initial pose if the given point is occupied in the map
  if (!isPointUnoccupied(init_pose_)) {
    init_pose_received_ = false;
    ROS_WARN(
      "The initial pose specified is on or too close to an obstacle please specify another point");
  } else {
    init_pose_received_ = true;

    drawGoalInitPose();
    ROS_INFO("Initial pose obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  if (goal_received_) {
    buildMapImage();
  }

  // Convert msg to Point2D
  poseToPoint(goal_, msg->pose);

  // Reject the goal pose if the given point is occupied in the map
  if (!isPointUnoccupied(goal_)) {
    goal_received_ = false;
    ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
  } else {
    goal_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Goal obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::drawGoalInitPose()
{
  if (goal_received_) {
    drawCircle(goal_, 3, cv::Scalar(12, 255, 43));
  }
  if (init_pose_received_) {
    drawCircle(init_pose_, 3, cv::Scalar(255, 200, 0));
  }
}

int RRTPlanner::euclideanDist(int x1, int y1, int x2, int y2){
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void RRTPlanner::plan(const int max_dist, const int search_r, const std::string variant)
{
  // Reset these values so planning only happens once for a
  // given pair of initial pose and goal points
  goal_received_ = false;
  init_pose_received_ = false;

  // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  //       path through the map starting from the initial pose and ending at the goal pose
  std::vector<std::pair<Point2D, int>> vertices;
  std::vector<std::vector<int>> edges;
  std::vector<int> edge(2);
  std::vector<int> lastVertices;
  std::vector<Point2D> completePath;
  std::vector<int> neighbours;
  int rand_x, rand_y;
  int min_dist;
  int dist;
  int nearest_i;
  int node_x, node_y;
  int cand_x, cand_y;
  int in_x, in_y, parent_x, parent_y, proximal_x, proximal_y;
  int radius;
  int x1, x2, x, y, y1, y2;
  int goal_x, goal_y, currIndex, i;
  int cost_x, cost_y, cost;
  float gradient, c;
  float theta;
  Point2D rand_pt, parent_pt, proximal_pt, candPoint, currPath, cost_pt, neighbour_pt;
  bool isOccupied;
  bool isComplete;
  bool infGrad;
  bool parentFound;
  
  completePath.clear();
  ROS_INFO("Computing  feasible path...");
  // Add the initial point as vertex.
  vertices.push_back({init_pose_, 0});

  goal_x = goal_.y();
  goal_y = map_grid_->info.height - goal_.x() - 1;
  
  isComplete = false;
  while(isComplete == false){
  //for(int k = 0; k < 50; k ++){
    // Generate a random point in the map.
    rand_x = rand() % map_grid_->info.width;
    rand_y = map_grid_->info.height - (rand() % map_grid_->info.height); 

    // Find the vertex that is nearest to the generated random point and make it as the parent vertex.
    nearest_i = 0;
    min_dist = 999999;
    for(int i = 0; i < vertices.size(); i++){
      node_y = map_grid_->info.height - vertices[i].first.x() - 1;
      node_x = vertices[i].first.y();

      dist = euclideanDist(rand_x, rand_y, node_x, node_y);
      if(dist < min_dist){
        nearest_i = i;
        min_dist = dist;
      }
    } 
    proximal_x = vertices[nearest_i].first.y();
    proximal_y = map_grid_->info.height - vertices[nearest_i].first.x() - 1;
    
    // Randomly generated point might be too far from the proximal vertex.
    // Set the new candidate point to a defined distance from the proximal vertex 
    // along the vector connecting the proximal vertex and the random point.
    radius = std::min(max_dist, min_dist);

    // Generate gradient between the proximal vertex and the candidate point.
    // Gradient will become infinity if proximal_x == candidate_x.
    // Boolean infGrad returns true if gradient becomes infinity.
    if (rand_x != proximal_x){
      infGrad = false;
      gradient = static_cast< float >(rand_y - proximal_y)/static_cast< float >(rand_x - proximal_x);
      c = proximal_y - (gradient * proximal_x);

      theta = atan(gradient);
      
      if(proximal_x < rand_x){
        cand_x = proximal_x + radius * cos(theta);
        cand_y = proximal_y + radius * sin(theta);
      }else{
        cand_x = proximal_x - radius * cos(theta);
        cand_y = proximal_y - radius * sin(theta);
      } 

    }else{
      infGrad = true;
      gradient = 0;
      c = 0;
      cand_x = proximal_x;
      cand_y = proximal_y + radius;
    }
    parent_x = proximal_x;
    parent_y = proximal_y;

    parent_pt.x(map_grid_->info.height - parent_y - 1);
    parent_pt.y(parent_x);

    candPoint.x(map_grid_->info.height - cand_y - 1);
    candPoint.y(cand_x);

    // Make sure that candidate point can be connected to its parent point without intersecting with obstacles.
    // If OK add the candidate point as vertex.
    
    if(variant == "rrt_star"){
      // Start of RRT*
      /**********************************************************************************************************************/
      // RRT* Part 1
      parentFound = false;
      min_dist = euclideanDist(cand_x, cand_y, parent_x, parent_y);
      neighbours.clear();
      if (isPointUnoccupied(candPoint)){
        // After candidate point is generated, search around the point for nearest vertex as parent vertex.
        for (int i = 0; i < vertices.size(); i++){
          cost_x = vertices[i].first.y();
          cost_y = map_grid_->info.height - vertices[i].first.x() - 1;
          cost = vertices[i].second;
          cost_pt.x(map_grid_->info.height - cost_y - 1);
          cost_pt.y(cost_x);

          if(euclideanDist(cand_x, cand_y, cost_x, cost_y) < search_r && euclideanDist(cand_x, cand_y, cost_x, cost_y) > 0 && isPointUnoccupied(cost_pt)){
            // Check if path is feasible first
            if(!isNotObstacleFree(cost_x, cost_y, cand_x, cand_y)){
              neighbours.push_back(i);
              if(cost < min_dist){
                parent_x = cost_x;
                parent_y = cost_y;
                min_dist = cost;
                parentFound = true;
                nearest_i = i;
              }
            }
          }
        }
        // In case no vertices have lower cost than the proximal vertex, proximal vertex is selected
        if(!isNotObstacleFree(parent_x, parent_y, cand_x, cand_y) && !parentFound){
          parentFound = true;
        }

        // If parent node can be found, candidate node is valid.
        if(parentFound){
          vertices.push_back({candPoint, min_dist});
          edge[0] = nearest_i;
          edge[1] = vertices.size() - 1;
          edges.push_back(edge);

          // RRT* Part 2
          int neighbour_x, neighbour_y, neighbour_cost, edge_i, prev_i, neigh_cand_dist;
          for(int i = 0; i < neighbours.size(); i++){
            Point2D neigh = vertices[i].first;
            // red - chosen point
            // green - candidate point
            // blue - neighbouring point
            drawCircle(neigh, 3, cv::Scalar(255, 0, 0));
            drawCircle(candPoint, 3, cv::Scalar(0, 255, 0));
            neighbour_x = vertices[i].first.y();
            neighbour_y = map_grid_->info.height - vertices[i].first.x() - 1;
            neigh_cand_dist = euclideanDist(cand_x, cand_y, neighbour_x, neighbour_y);

            prev_i = nearest_i;
            int j = edges.size() - 1;
            int totCost = vertices[edges[edges.size() - 1][1]].second;
            while (prev_i != i and j >= 0){
              if(edges[j][1] == prev_i){
                prev_i = edges[j][0];
                totCost += vertices[edges[j][1]].second;
                drawCircle(vertices[edges[j][1]].first, 3, cv::Scalar(0, 0, 255));
              }
              j --;
            }
            if(neigh_cand_dist < totCost && !isNotObstacleFree(neighbour_x, neighbour_y, cand_x, cand_y)){
              edges[edges.size() - 1][0] = i;
              vertices[vertices.size() - 1].second = neigh_cand_dist;
            }

          }
          // Check whether the last vertex can be connected to the goal point via a straight line without intersecting with obstacles.
          // If OK end the loop
          int cand_goal_dist = euclideanDist(cand_x, cand_y, goal_x, goal_y);
          if(cand_goal_dist < max_dist){
            infGrad = false;
            
            if(!isNotObstacleFree(goal_x, goal_y, cand_x, cand_y)){
              isComplete = true;
              edge[0] = vertices.size() - 1;
              vertices.push_back({goal_, cand_goal_dist});
              edge[1] = vertices.size() - 1;
              edges.push_back(edge);
            }
          }
        }
      }  
      /**********************************************************************************************************************/
      // End of RRT*
    }
    else if(variant == "rrt_vanilla")    
    {
      // Start of vanilla RRT
      /**********************************************************************************************************************/
      if (isPointUnoccupied(candPoint)){
        if(!isNotObstacleFree(parent_x, parent_y, cand_x, cand_y)){
          drawCircle(candPoint, 3, cv::Scalar(0, 255, 0));
          int cand_parent_dist = euclideanDist(cand_x, cand_y, parent_x, parent_y);
          vertices.push_back({candPoint, cand_parent_dist});
          edge[0] = nearest_i;
          edge[1] = vertices.size() - 1;
          edges.push_back(edge);
          // Check whether the last vertex can be connected to the goal point via a straight line without intersecting with obstacles.
          // If OK end the loop
          int cand_goal_dist = euclideanDist(cand_x, cand_y, goal_x, goal_y);
          if(cand_goal_dist < max_dist){
            infGrad = false;
            if (goal_x != cand_x){
              infGrad = false;
              gradient = static_cast< float >(goal_y - cand_y)/static_cast< float >(goal_x - cand_x);
              c = cand_y - (gradient * cand_x);
            }else{
              infGrad = true;
              gradient = 0;
              c = 0;
            }
            
            if(!isNotObstacleFree(goal_x, goal_y, cand_x, cand_y)){
              isComplete = true;
              //drawLine(goal_, candPoint, cv::Scalar(255, 0, 0), 1);
              edge[0] = vertices.size() - 1;
              //test
              vertices.push_back({goal_, cand_goal_dist});
              edge[1] = vertices.size() - 1;
              //edge[1] = 1;
              //std::cout <<"edge: ("<< edge[0]<<", " << edge[1]<<") \n";
              edges.push_back(edge);
            }
          }
        }
      }
      /**********************************************************************************************************************/
      // End of vanilla RRT
    }
  }

  ROS_INFO("Determining path..");
  // Compute the path connecting initail position and goal position
  currIndex = edges[edges.size() - 1][1];;
  currPath = vertices[currIndex].first;
  drawCircle(currPath, 3, cv::Scalar(255, 0, 127));
  completePath.push_back(currPath);

  i = edges.size() - 1;
  while((currPath.x() != init_pose_.x() || currPath.y() != init_pose_.y()) && i >= 0){
    if(currIndex == edges[i][1]){
      currIndex = edges[i][0];
      currPath = vertices[currIndex].first;
      completePath.insert(completePath.begin(), currPath);
    }
    i--;
  }
  for (int j = 0; j < edges.size(); j ++){
    drawLine(vertices[edges[j][0]].first, vertices[edges[j][1]].first, cv::Scalar(255, 0, 0), 1);
  }
  
  publishPath(completePath);

  ROS_INFO("Path computed succesfully");
}

void RRTPlanner::publishPath(std::vector<Point2D> completePath)
{
  // Create new Path msg
  nav_msgs::Path path;
  path.header.frame_id = map_grid_->header.frame_id;
  path.header.stamp = ros::Time::now();

  // TODO: Fill nav_msgs::Path msg with the path calculated by RRT
  for (int i=0; i<completePath.size(); i++){
    geometry_msgs::PoseStamped pose = pointToPose(completePath[i]);
    if (i > 0){
      // Draw line on output
      drawLine(completePath[i], completePath[i - 1], cv::Scalar(255, 0, 255), 3);
    }
    path.poses.push_back(pose);
  }
  // Publish the calculated path
  path_pub_.publish(path);

  displayMapImage();
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  // TODO: Fill out this function to check if a given point is occupied/free in the map
  int x = std::max(p.y(), 0);
  int y = std::max(p.x(), 0);
  int occupancy = map_grid_->data[x + map_grid_->info.width * y];

  if(occupancy != 0){
    return false;
  }else{
    return true;
  }
}

bool RRTPlanner::isNotObstacleFree(int start_x, int start_y, int end_x, int end_y)//, bool infGrad, float gradient, float c)
{
  bool isOccupied = false;
  int x1, x2, y1, y2, x, y;
  Point2D testPoint;
  float gradient, c;
  bool infGrad = false;
  if (start_x != end_x){
    infGrad = false;
    gradient = static_cast< float >(end_y - start_y)/static_cast< float >(end_x - start_x);
    c = start_y - (gradient * start_x);
  }else{
    infGrad = true;
    gradient = 0;
    c = 0;
  }
  // Incrementing the value of x to get the coordinates of each point
  // along the straight line between start vertex and end vertex.  
  // When gradient is too steep (abs(gradient) > 1),  i.e. y >> x, need to increment y value instead of x as x becomes insensitive to changes
  if(!infGrad && abs(gradient) <= 1){
    if (start_x < end_x){
      x1 = start_x;
      x2 = end_x;
    }
    else
    {
      x1 = end_x;
      x2 = start_x;
    }
    x = x1;
    while(x < x2 && isOccupied == false){
      y = gradient * x + c;
      testPoint.x(map_grid_->info.height - y - 1);
      testPoint.y(x);
      if (!isPointUnoccupied(testPoint)){
        isOccupied = true;
      }
      else
      {
        isOccupied = false;
      }
      x ++;
    }
  }
  else
  {
    if (start_y < end_y){
      y1 = start_y;
      y2 = end_y;
    }
    else
    {
      y1 = end_y;
      y2 = start_y;
    }
    y = y1;
    while(y < y2 && isOccupied == false){
      if(infGrad){
        x = end_x;
      }
      else
      {
        x = (y - c)/gradient;
      }
      testPoint.x(map_grid_->info.height - y - 1);
      testPoint.y(x);
      if (!isPointUnoccupied(testPoint)){
        isOccupied = true;
      }
      else
      {
        isOccupied = false;
      }
      y ++;
    }
  }

  return isOccupied;
}

void RRTPlanner::buildMapImage()
{
  // Create a new opencv matrix with the same height and width as the received map
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  // Fill the opencv matrix pixels with the map points
  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[toIndex(i, j)]) {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
  cv::line(
    *map_,
    cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
    cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
    color,
    thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y)
{
  return x * map_grid_->info.width + y;
}



} // rrt_planner