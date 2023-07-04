#include "rrt_planner/mapping.h"

namespace mapping 
{
cv::Mat src;
cv::Point point;
std::vector<cv::Point> pts;
std::vector<std::string> folder;

int drag = 0;
int var = 0;
int flag = 0;
int ix, iy;
int c;
bool drawing = false;
bool erase = false;
bool save = false;
bool isClose = false;

Mapping::Mapping()
{
  // Initialize node handles
  node_.reset(new ros::NodeHandle());
  node_params_.reset(new ros::NodeHandle("~"));
}

void Mapping::run()
{
  node_->param<std::string>("resources_dir", resources_dir_, "/src/rrt_planner/resources");
  node_->param<std::string>("cfg_dir", cfg_dir_, "/src/rrt_planner/cfg");
  node_params_->param<int>("width", width_, 512);
  node_params_->param<int>("height", height_, 512);

  ROS_INFO("Map Editor Node");
  ROS_INFO("The Map Editor allows you to draw custom maps that can be used in the RRT node.");
  ROS_INFO("resources dir: %s", resources_dir_.c_str());
  ROS_INFO("cfg dir: %s", cfg_dir_.c_str());
  ROS_INFO("Map size: %d x %d", width_, height_);
  ROS_INFO("Left-click and drag to draw");
  ROS_INFO("Middle-click and drag to erase");
  ROS_INFO("Right-click to clear all");
  ROS_INFO("Press 'e' to save");
  ROS_INFO("Image will be saved to ~/src/rrt_planner_ros/resources");
  ROS_INFO("Right-click to clear all");

  src = cv::Mat(cv::Size(width_,height_),CV_8UC3,cv::Scalar::all(255));

  cv::namedWindow("Map Editor - Press 'e' to save", cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback("Map Editor - Press 'e' to save", Mapping::mouseHandler, NULL);
  cv::imshow("Map Editor - Press 'e' to save", src);

  while(!isClose){
    c = cv::waitKey(); 
    if(c == 101){
      save = true;
      ROS_INFO("Saving...");
      isClose = true;
    }else if(c == -1){
      isClose = true;
    }
  }
  
  if(save){
    // std::string path_string=std::filesystem::current_path();
    // std::string path = path_string + "/src/rrt_planner_ros/resources";
    // std::cout << "Current path is " << path << '\n';
    for (const auto & entry : std::filesystem::directory_iterator(resources_dir_)){
        folder.push_back(entry.path());
    }

    cv::imwrite(resources_dir_ + "/map" + std::to_string(folder.size() + 1) + ".png", src);
    ROS_INFO("Image: map %s.png saved to %s", std::to_string(folder.size() + 1).c_str(), resources_dir_.c_str());

    YAML::Node map_node;

    map_node["image"] = resources_dir_ + "/map" + std::to_string(folder.size() + 1) + ".png";
    map_node["resolution"] = "0.1";
    map_node["origin"] = YAML::Load("[0.0, 0.0, 0.0]");
    map_node["occupied_thresh"] = "0.65";
    map_node["free_thresh"] = "0.196";
    map_node["negate"] = "0";

    std::ofstream fout(cfg_dir_ + "/map" + std::to_string(folder.size() + 1) + ".yaml"); 
    fout << map_node;
    //std::cout<<"Image: map" << folder.size() << ".png saved to "<<resources_dir_<<"\n";
  }

  cv::destroyAllWindows();
  cv::waitKey(1);
  ROS_INFO("Window closed.");
}

void Mapping::mouseHandler(int event, int x, int y, int, void*)
{

  if (event == cv::EVENT_LBUTTONDOWN){
    drawing = true;
    pts.push_back(cv::Point(x, y));
    ix = x;
    iy = y;
    //cv::circle(src, point, 5, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::imshow("Map Editor - Press 'e' to save", src);
  }
  if(event == cv::EVENT_MOUSEMOVE){
    if(drawing){
        pts.push_back(cv::Point(x, y));
        cv::line(src, cv::Point(ix, iy), cv::Point(x, y), cv::Scalar(0, 0, 0), 5);
        cv::imshow("Map Editor - Press 'e' to save", src);
        ix = x;
        iy = y;
    }
    if(erase){
        cv::line(src, cv::Point(ix, iy), cv::Point(x, y), cv::Scalar(255, 255, 255), 5);
        cv::imshow("Map Editor - Press 'e' to save", src);
        ix = x;
        iy = y;
    }
      
  }
  if(event == cv::EVENT_LBUTTONUP){
    drawing = false;
    pts.push_back(cv::Point(x, y));
    cv::line(src, cv::Point(ix, iy), cv::Point(x, y), cv::Scalar(0, 0, 0), 5); 
    cv::imshow("Map Editor - Press 'e' to save", src);
  }
  if(event == cv::EVENT_RBUTTONDOWN){
    src = cv::Mat(cv::Size(512,512),CV_8UC3,cv::Scalar::all(255));
    drawing = false;
    pts.clear();
    cv::imshow("Map Editor - Press 'e' to save", src);
  }
  if (event == cv::EVENT_MBUTTONDOWN){
    erase = true;
    pts.push_back(cv::Point(x, y));
    ix = x;
    iy = y;
    //cv::circle(src, point, 5, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::imshow("Map Editor - Press 'e' to save", src);
  }
  if(event == cv::EVENT_MBUTTONUP){
    erase = false;
    cv::line(src, cv::Point(ix, iy), cv::Point(x, y), cv::Scalar(255, 255, 255), 5); 
    cv::imshow("Map Editor - Press 'e' to save", src);
  }

}

}