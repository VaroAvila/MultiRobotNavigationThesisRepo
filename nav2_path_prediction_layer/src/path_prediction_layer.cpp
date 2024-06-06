#include "nav2_path_prediction_layer/path_prediction_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include <vector>
#include <cmath>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_path_prediction_layer_plugin
{

PathPredictionLayer::PathPredictionLayer():
  last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

void
PathPredictionLayer::onInitialize()
{
  std::string robot_name;
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  declareParameter("robot_name", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + "." + "robot_name", robot_name);
  
  subscribed_topic_plan_ = "/" + robot_name + "/plan";
  subscribed_topic_cmd_vel_ = "/" + robot_name + "/cmd_vel";


  need_recalculation_ = false;
  current_ = true;

  robot_path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
  subscribed_topic_plan_, 1, std::bind(&PathPredictionLayer::path_callback, this, std::placeholders::_1));

  robot_cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
  subscribed_topic_cmd_vel_, 1, std::bind(&PathPredictionLayer::cmd_vel_callback, this, std::placeholders::_1));

}

void PathPredictionLayer::path_callback(const nav_msgs::msg::Path::SharedPtr path)
{
  robot_path_ = path;
}

//Changes the length of the published path based on the speed. In this case its discreet to make the code simpler. 

void PathPredictionLayer::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
  if(cmd_vel)
  {
    robot_cmd_vel_ = cmd_vel;
    linear_speed = calculateSpeed(*robot_cmd_vel_);
    while(true){
    if(linear_speed < 0.1)
      {
      path_length = 50;
      break;
      }
    else if(linear_speed < 0.2)
      {
      path_length = 100;
      break;
      }
    else if(linear_speed < 0.3)
      {
      path_length = 200;
      break;
      }
    else
      {
      path_length = 250;
      break;
      }
    }
  }
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
PathPredictionLayer::updateBounds(
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

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
PathPredictionLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "PathPredictionLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
PathPredictionLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{   
// EL ERROR ES EL WORLD TO MAP!!!

     unsigned int aux_x, aux_y;

    if (!stored_segment_.empty()) {
      for (const auto& path_point_info : stored_segment_)
      {
        master_grid.setCost(path_point_info.x, path_point_info.y, FREE_SPACE);
      }  // FREE SPACE MAKE SENSE AS THE PATH WILL NEVER BE PLANNED THROUGH AN OCUPPIED SPACE/OBSTACLE.
         // THE ONLY CASE WHERE IT COULD AFFECT IS WITH DYNAMIC OBSTACLES BUT IN THAT CASE, EACH MOVEMENT 
         // OF THE OBSTACLE WILL BE SHOWN. 
      stored_segment_.clear();
      }
    
    if (!enabled_ || !robot_path_ || static_cast<int>(robot_path_->poses.size()) < path_length) { //No tiene por que ser path_length como limite necesariamente, puede ser p.ej. la mitad.
      
      return;
    }

    for(int i=0; i<path_length; i++){

      if (!master_grid.worldToMap(robot_path_->poses[i].pose.position.x, robot_path_->poses[i].pose.position.y, aux_x, aux_y)){
        return;
      }

        point_cost_info.x = aux_x;
        point_cost_info.y = aux_y;
        point_cost_info.cost = master_grid.getCost(aux_x, aux_y);
        stored_segment_.emplace_back(point_cost_info);

        master_grid.setCost(point_cost_info.x, point_cost_info.y, LETHAL_OBSTACLE);  // Set the cost of the cell

        // Anadir un if que rompa el bucle si se acaban las posiciones de la lista
    
    }

  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

}

double 
PathPredictionLayer::calculateSpeed(const geometry_msgs::msg::Twist& twist) {

    double linear_speed = std::sqrt(
        std::pow(twist.linear.x, 2) + 
        std::pow(twist.linear.y, 2) + 
        std::pow(twist.linear.z, 2));
    return linear_speed;
}

}  // namespace nav2_path_prediction_layer_plugin


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_path_prediction_layer_plugin::PathPredictionLayer, nav2_costmap_2d::Layer)
