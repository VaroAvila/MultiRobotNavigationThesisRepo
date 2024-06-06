#ifndef PATH_PREDICTION_LAYER_HPP_
#define PATH_PREDICTION_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace nav2_path_prediction_layer_plugin
{

class PathPredictionLayer : public nav2_costmap_2d::Layer
{
public:
  PathPredictionLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  nav_msgs::msg::Path::SharedPtr robot_path_;
  geometry_msgs::msg::Twist::SharedPtr robot_cmd_vel_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr robot_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_cmd_vel_sub_;

  virtual void reset()
  {
    return;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr);

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

  double calculateSpeed(const geometry_msgs::msg::Twist& twist);


  struct SegmentPointData {
    unsigned int x;
    unsigned int y;
    unsigned char cost;
  };

  // Struct to store the values of a point of the map and its cost. Right now the stored cost is not used for this node, but it was in previous versions
  // and it is left in case the code is extended 
  SegmentPointData point_cost_info; 

  std::string subscribed_topic_plan_;
  std::string subscribed_topic_cmd_vel_;

  std::vector<SegmentPointData> stored_segment_;

  int path_length = 300;
  double linear_speed;

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  bool need_recalculation_;

};

}  // namespace nav2_path_prediction_layer_plugin

#endif  // PATH_PREDICTION_LAYER_HPP_