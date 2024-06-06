#include "nav2_footprint_costmap_plugin/footprint_layer.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;


namespace nav2_footprint_costmap_plugin
{

FootprintLayer::FootprintLayer():
  last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}


void
FootprintLayer::onInitialize()
{
  std::string robot_name;
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  declareParameter("robot_name", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + "." + "robot_name", robot_name);
  
  subscribed_topic = "/" + robot_name + "/global_costmap/published_footprint";
  need_recalculation_ = false;
  current_ = true;

  robot_footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      subscribed_topic, 10, std::bind(&FootprintLayer::footprint_callback, this, std::placeholders::_1));
}


void 
FootprintLayer::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr robot_footprint)
{
  robot_footprint_ = robot_footprint;
}


void
FootprintLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}


void
FootprintLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "FootprintLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}


void
FootprintLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{   

    if (!enabled_ || !robot_footprint_ || robot_footprint_->polygon.points.size() < 3) {
      return;
    }

    robot_footprint_derreferenced = *robot_footprint_;
    int next_polygon_point;

    if (!stored_segment_.empty()) {
      for (const auto& segment_point_info : stored_segment_){
        master_grid.setCost(segment_point_info.x, segment_point_info.y, FREE_SPACE);
      }
      stored_segment_.clear();
    }

    for (int polygon_point_index = 0; polygon_point_index < static_cast<int>(robot_footprint_->polygon.points.size()); polygon_point_index++)
    {
      next_polygon_point = polygon_point_index + 1;

      if (next_polygon_point == static_cast<int>(robot_footprint_->polygon.points.size())) 
      {
        next_polygon_point = 0;
      }

      drawSegment(master_grid, 
                  robot_footprint_derreferenced.polygon.points[polygon_point_index].x, 
                  robot_footprint_derreferenced.polygon.points[polygon_point_index].y, 
                  robot_footprint_derreferenced.polygon.points[next_polygon_point].x, 
                  robot_footprint_derreferenced.polygon.points[next_polygon_point].y,  
                  LETHAL_OBSTACLE, 
                  stored_segment_
                  ); 
    }

  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);
}

void 
FootprintLayer::drawSegment(
  nav2_costmap_2d::Costmap2D & master_grid, double x0, double y0, double x1, double y1, unsigned char cost, std::vector<SegmentPointData>& stored_segment)
{
    unsigned int mx0, my0, mx1, my1;

    if (!master_grid.worldToMap(x0, y0, mx0, my0) || !master_grid.worldToMap(x1, y1, mx1, my1)) {
        return;  
    }
    // Use Bresenham's line algorithm to set the cost of cells along the line - transforms a continuous segment into a cell based segment
    int dx = abs(static_cast<int>(mx1 - mx0));
    int dy = abs(static_cast<int>(my1 - my0));
    int sx = (mx0 < mx1) ? 1 : -1;
    int sy = (my0 < my1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        point_cost_info.x = mx0;
        point_cost_info.y = my0;
        point_cost_info.cost = master_grid.getCost(mx0, my0);
        stored_segment.emplace_back(point_cost_info);
        master_grid.setCost(mx0, my0, cost); 

        if (mx0 == mx1 && my0 == my1) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err = err - dy;
            mx0 = mx0 + sx;
        }
        if (e2 < dx) {
            err = err + dx;
            my0 = my0 + sy;
        }
    }
}

}  // namespace nav2_footprint_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_footprint_costmap_plugin::FootprintLayer, nav2_costmap_2d::Layer)

