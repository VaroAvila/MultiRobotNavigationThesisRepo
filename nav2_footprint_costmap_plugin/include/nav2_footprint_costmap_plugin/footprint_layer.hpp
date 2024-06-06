#ifndef FOOTPRINT_LAYER_HPP_
#define FOOTPRINT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <vector>
#include <cmath>
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace nav2_footprint_costmap_plugin
{

class FootprintLayer : public nav2_costmap_2d::Layer
{
public:
  FootprintLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  geometry_msgs::msg::PolygonStamped::SharedPtr robot_footprint_;

  geometry_msgs::msg::PolygonStamped robot_footprint_derreferenced;

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr robot_footprint_sub_;

  virtual void reset()
  {
    return;
  }

  void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr);

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

  struct SegmentPointData {
    unsigned int x;
    unsigned int y;
    unsigned char cost;
  };

  SegmentPointData point_cost_info;
  std::string subscribed_topic;
  std::vector<SegmentPointData> stored_segment_;  // Vector to store the footprint segments modified cells
  void drawSegment(nav2_costmap_2d::Costmap2D & master_grid, double x0, double y0, double x1, double y1, unsigned char cost, std::vector<SegmentPointData>& stored_segment);

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_recalculation_;
};

}  // namespace nav2_footprint_costmap_plugin

#endif  // FOOTPRINT_LAYER_HPP_
