/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// CODE EXTENDED BY ALVARO AVILA

#include "../include/NeoFleetRViz2Plugin.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>

#include "string"
#include "map"
#include "vector"

#include "QPainter"
#include "QLineEdit"
#include "QVBoxLayout"
#include "QHBoxLayout"
#include "QLabel"
#include "QTimer"
#include "QComboBox"
#include "QFrame"
#include "QCheckBox"

namespace neo_fleet
{

// --- CONSTRUCTOR ---
Worker::Worker()
{
  /** Reserving the vector size of the robots to the expected
   * number of robots provided by the user **/
  robots_.reserve(available_robots_.size());
  robot_namespaces_.reserve(available_robots_.size());

}

// --- DECONSTRUCTOR ---
Worker::~Worker()
{
}

void Worker::checkAndStoreRobot(const std::string & robot)
{
  for (int i = 0; i < available_robots_.size(); ++i) {
    if (robot == available_robots_[i]) {
      robot_namespaces_.push_back(robot);
    }
  }
}

// --- PROCESS ---
// Start processing data.
void Worker::process()
{
  std::map<std::string, std::vector<std::string>> get_topic = node_->get_topic_names_and_types();
  std::string robots = "";
  std::string tmp = "";

  // Store the robots for the drop down list
  for (auto it = get_topic.begin(); it != get_topic.end(); it++) {
    int dslash = 0;
    for (int i = 0; i < (it->first).size(); i++) {
      if (it->first[i] != '/') {
        robots += it->first[i];
      }
      if (it->first[i] == '/' && dslash != 2) {
        dslash++;
        if (dslash == 2 && tmp != robots) {
          checkAndStoreRobot(robots);
          tmp = robots;
        }
      }
    }
    robots.clear();
  }

  // Allocating ros helpers depending on the number of robots available.
  if (robot_namespaces_.size() == 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "There are no robots available"
    );
    return;
  }

  if (robots_.size() < robot_namespaces_.size()) {
    robots_.resize(robot_namespaces_.size());
  }

  for (int i = 0; i < robots_.size(); i++) {
    robots_[i] = std::make_shared<RosHelper>(node_, robot_namespaces_[i]);
    robot_identity_map_.insert(
      std::pair<std::string, std::shared_ptr<RosHelper>>(
        robot_namespaces_[i],
        robots_[i]));
  }

  initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1, std::bind(&Worker::pose_callback, this, std::placeholders::_1));
  goal_pos_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 1, std::bind(&Worker::goal_callback, this, std::placeholders::_1));
  

  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok()) {
    loop_rate.sleep();
    rclcpp::spin_some(node_);
    emit send_pos();
  }
}

NeoFleetRViz2Plugin::NeoFleetRViz2Plugin(QWidget * parent)
: rviz_common::Panel(parent), server_timeout_(100)
{
  client_node_ = std::make_shared<rclcpp::Node>("__");
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(client_node_->get_clock());
  // Nav2GoalPublisher nav2GoalCaller;
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    client_node_->get_node_base_interface(),
    client_node_->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  main_layout_ = new QVBoxLayout;
  top_layout_ = new QHBoxLayout;
  topic_layout_ = new QHBoxLayout;
  output_status_editor_ = new QLineEdit;
  warn_signal_ = new QLabel("No robot selected");

  // robot0_replanning_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
  //   "robot0/robot_replanning_status_topic", 1, std::bind(&Worker::robot0_replanning_status_callback, this, std::placeholders::_1));
  // robot1_replanning_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
  //   "robot1/robot_replanning_status_topic", 1, std::bind(&Worker::robot1_replanning_status_callback, this, std::placeholders::_1));
  // robot2_replanning_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
  //   "robot2/robot_replanning_status_topic", 1, std::bind(&Worker::robot2_replanning_status_callback, this, std::placeholders::_1));

  robots_layout_ = new QVBoxLayout;
  robot0_layout_ = new QHBoxLayout;
  robot1_layout_ = new QHBoxLayout;
  robot2_layout_ = new QHBoxLayout;

  robot_goals_container_0_ = new QComboBox(this);
  robot_goals_container_1_ = new QComboBox(this);
  robot_goals_container_2_ = new QComboBox(this);

  connect(robot_goals_container_0_, QOverload<int>::of(&QComboBox::activated), this, &NeoFleetRViz2Plugin::setRobotName);
  connect(robot_goals_container_1_, QOverload<int>::of(&QComboBox::activated), this, &NeoFleetRViz2Plugin::setRobotName);
  connect(robot_goals_container_2_, QOverload<int>::of(&QComboBox::activated), this, &NeoFleetRViz2Plugin::setRobotName);

  //! Crear un vector con los datos del nombre y las coordenadas, y sustituir esto por un bucle for que itere el size_of vector y haga un for(const auto& members : vector){robot_container->addItem(members.name)} 

  // for (const auto& location : location_vector)
  // {
  //     robot_goals_container_0_->addItem(location.name);
  //     robot_goals_container_1_->addItem(location.name);
  //     robot_goals_container_2_->addItem(location.name);
  // }

// SINGLE ROAD LOCATION DATA

  // LocationData Elevator0;
  //   Elevator0.name = "Elevator0";
  //   Elevator0.pose_stamped.header.frame_id = "map"; 
  //   Elevator0.pose_stamped.pose.position.x = 25.3; 
  //   Elevator0.pose_stamped.pose.position.y = 23.0; 
  //   Elevator0.pose_stamped.pose.position.z = 0.0;
  //   Elevator0.pose_stamped.pose.orientation.x = 0.0; 
  //   Elevator0.pose_stamped.pose.orientation.y = 0.0; 
  //   Elevator0.pose_stamped.pose.orientation.z = 1.0;
  //   Elevator0.pose_stamped.pose.orientation.w = 0.0; 

  //   stored_goal_locations_.emplace_back(Elevator0);

  // LocationData Elevator1;
  //   Elevator1.name = "Elevator1";
  //   Elevator1.pose_stamped.header.frame_id = "map"; 
  //   Elevator1.pose_stamped.pose.position.x = 25.3; 
  //   Elevator1.pose_stamped.pose.position.y = 21.0; 
  //   Elevator1.pose_stamped.pose.position.z = 0.0;
  //   Elevator1.pose_stamped.pose.orientation.x = 0.0; 
  //   Elevator1.pose_stamped.pose.orientation.y = 0.0; 
  //   Elevator1.pose_stamped.pose.orientation.z = 1.0;
  //   Elevator1.pose_stamped.pose.orientation.w = 0.0; 

  //   stored_goal_locations_.emplace_back(Elevator1);

  // LocationData Elevator2;
  //   Elevator2.name = "Elevator2";
  //   Elevator2.pose_stamped.header.frame_id = "map"; 
  //   Elevator2.pose_stamped.pose.position.x = 25.3; 
  //   Elevator2.pose_stamped.pose.position.y = 19.0; 
  //   Elevator2.pose_stamped.pose.position.z = 0.0;
  //   Elevator2.pose_stamped.pose.orientation.x = 0.0; 
  //   Elevator2.pose_stamped.pose.orientation.y = 0.0; 
  //   Elevator2.pose_stamped.pose.orientation.z = 1.0;
  //   Elevator2.pose_stamped.pose.orientation.w = 0.0; 

  //   stored_goal_locations_.emplace_back(Elevator2);


  LocationData SR_L_0;
    SR_L_0.name = "SR_L_0";
    SR_L_0.pose_stamped.header.frame_id = "map"; 
    SR_L_0.pose_stamped.pose.position.x = -14.0; 
    SR_L_0.pose_stamped.pose.position.y = 0.0; 
    SR_L_0.pose_stamped.pose.position.z = 0.0;
    SR_L_0.pose_stamped.pose.orientation.x = 0.0; 
    SR_L_0.pose_stamped.pose.orientation.y = 0.0; 
    SR_L_0.pose_stamped.pose.orientation.z = 1.0;
    SR_L_0.pose_stamped.pose.orientation.w = 0.0; 

    stored_goal_locations_.emplace_back(SR_L_0);

  LocationData SR_R_0;
    SR_R_0.name = "SR_R_0";
    SR_R_0.pose_stamped.header.frame_id = "map"; 
    SR_R_0.pose_stamped.pose.position.x = 14.0; 
    SR_R_0.pose_stamped.pose.position.y = 0.0; 
    SR_R_0.pose_stamped.pose.position.z = 0.0;
    SR_R_0.pose_stamped.pose.orientation.x = 0.0; 
    SR_R_0.pose_stamped.pose.orientation.y = 0.0; 
    SR_R_0.pose_stamped.pose.orientation.z = 0.0;
    SR_R_0.pose_stamped.pose.orientation.w = 0.0; 

    stored_goal_locations_.emplace_back(SR_R_0);

  LocationData SR_L_TOP;
    SR_L_TOP.name = "SR_L_TOP";
    SR_L_TOP.pose_stamped.header.frame_id = "map"; 
    SR_L_TOP.pose_stamped.pose.position.x = -14.0; 
    SR_L_TOP.pose_stamped.pose.position.y = 2.0; 
    SR_L_TOP.pose_stamped.pose.position.z = 0.0;
    SR_L_TOP.pose_stamped.pose.orientation.x = 0.0; 
    SR_L_TOP.pose_stamped.pose.orientation.y = 0.0; 
    SR_L_TOP.pose_stamped.pose.orientation.z = 0.0;    
    SR_L_TOP.pose_stamped.pose.orientation.w = 0.0; 

    stored_goal_locations_.emplace_back(SR_L_TOP);

  LocationData SR_L_BOTTOM;
    SR_L_BOTTOM.name = "SR_L_BOTTOM";
    SR_L_BOTTOM.pose_stamped.header.frame_id = "map"; 
    SR_L_BOTTOM.pose_stamped.pose.position.x = -14.0; 
    SR_L_BOTTOM.pose_stamped.pose.position.y = -2.0; 
    SR_L_BOTTOM.pose_stamped.pose.position.z = 0.0;
    SR_L_BOTTOM.pose_stamped.pose.orientation.x = 0.0; 
    SR_L_BOTTOM.pose_stamped.pose.orientation.y = 0.0; 
    SR_L_BOTTOM.pose_stamped.pose.orientation.z = 0.0;
    SR_L_BOTTOM.pose_stamped.pose.orientation.w = 0.0; 

    stored_goal_locations_.emplace_back(SR_L_BOTTOM);


// CROSS ROAD LOCATION DATA

  LocationData CR_L_0;
    CR_L_0.name = "CR_L_0";
    CR_L_0.pose_stamped.header.frame_id = "map"; 
    CR_L_0.pose_stamped.pose.position.x = -12.0; 
    CR_L_0.pose_stamped.pose.position.y = 0.0; 
    CR_L_0.pose_stamped.pose.position.z = 0.0;
    CR_L_0.pose_stamped.pose.orientation.x = 0.0; 
    CR_L_0.pose_stamped.pose.orientation.y = 0.0; 
    CR_L_0.pose_stamped.pose.orientation.z = 1.0;
    CR_L_0.pose_stamped.pose.orientation.w = 0.0; 

    stored_goal_locations_.emplace_back(CR_L_0);
   
  LocationData CR_R_0;
    CR_R_0.name = "CR_R_0";
    CR_R_0.pose_stamped.header.frame_id = "map"; 
    CR_R_0.pose_stamped.pose.position.x = 12.0; 
    CR_R_0.pose_stamped.pose.position.y = 0.0; 
    CR_R_0.pose_stamped.pose.position.z = 0.0;
    CR_R_0.pose_stamped.pose.orientation.x = 0.0; 
    CR_R_0.pose_stamped.pose.orientation.y = 0.0; 
    CR_R_0.pose_stamped.pose.orientation.z = 0.0;
    CR_R_0.pose_stamped.pose.orientation.w = 0.0; 

    stored_goal_locations_.emplace_back(CR_R_0);

  LocationData CR_UP_0;
    CR_UP_0.name = "CR_UP_0";
    CR_UP_0.pose_stamped.header.frame_id = "map"; 
    CR_UP_0.pose_stamped.pose.position.x = 0.0; 
    CR_UP_0.pose_stamped.pose.position.y = 12.0; 
    CR_UP_0.pose_stamped.pose.position.z = 0.0;
    CR_UP_0.pose_stamped.pose.orientation.x = 0.0; 
    CR_UP_0.pose_stamped.pose.orientation.y = 0.0; 
    CR_UP_0.pose_stamped.pose.orientation.z = 0.7071067811865475;
    CR_UP_0.pose_stamped.pose.orientation.w = 0.7071067811865475; 

    stored_goal_locations_.emplace_back(CR_UP_0);

  LocationData CR_DOWN_0;
    CR_DOWN_0.name = "CR_DOWN_0";
    CR_DOWN_0.pose_stamped.header.frame_id = "map"; 
    CR_DOWN_0.pose_stamped.pose.position.x = 0.0; 
    CR_DOWN_0.pose_stamped.pose.position.y = -12.0; 
    CR_DOWN_0.pose_stamped.pose.position.z = 0.0;
    CR_DOWN_0.pose_stamped.pose.orientation.x = 0.0; 
    CR_DOWN_0.pose_stamped.pose.orientation.y = 0.0; 
    CR_DOWN_0.pose_stamped.pose.orientation.z = 0.7071067811865475;
    CR_DOWN_0.pose_stamped.pose.orientation.w = -0.7071067811865475; 

    stored_goal_locations_.emplace_back(CR_DOWN_0);

  LocationData CR_DOWN_LEFT;
    CR_DOWN_LEFT.name = "CR_DOWN_LEFT";
    CR_DOWN_LEFT.pose_stamped.header.frame_id = "map"; 
    CR_DOWN_LEFT.pose_stamped.pose.position.x = -3.0; 
    CR_DOWN_LEFT.pose_stamped.pose.position.y = -12.0; 
    CR_DOWN_LEFT.pose_stamped.pose.position.z = 0.0;
    CR_DOWN_LEFT.pose_stamped.pose.orientation.x = 0.0; 
    CR_DOWN_LEFT.pose_stamped.pose.orientation.y = 0.0; 
    CR_DOWN_LEFT.pose_stamped.pose.orientation.z = 0.7071067811865475;
    CR_DOWN_LEFT.pose_stamped.pose.orientation.w = -0.7071067811865475; 

    stored_goal_locations_.emplace_back(CR_DOWN_LEFT);

  LocationData CR_DOWN_RIGHT;
    CR_DOWN_RIGHT.name = "CR_DOWN_RIGHT";
    CR_DOWN_RIGHT.pose_stamped.header.frame_id = "map"; 
    CR_DOWN_RIGHT.pose_stamped.pose.position.x = 3.0; 
    CR_DOWN_RIGHT.pose_stamped.pose.position.y = -12.0; 
    CR_DOWN_RIGHT.pose_stamped.pose.position.z = 0.0;
    CR_DOWN_RIGHT.pose_stamped.pose.orientation.x = 0.0; 
    CR_DOWN_RIGHT.pose_stamped.pose.orientation.y = 0.0; 
    CR_DOWN_RIGHT.pose_stamped.pose.orientation.z = 0.7071067811865475;
    CR_DOWN_RIGHT.pose_stamped.pose.orientation.w = -0.7071067811865475; 

    stored_goal_locations_.emplace_back(CR_DOWN_RIGHT);

  LocationData CR_R_TOP;
    CR_R_TOP.name = "CR_R_TOP";
    CR_R_TOP.pose_stamped.header.frame_id = "map"; 
    CR_R_TOP.pose_stamped.pose.position.x = 12.0; 
    CR_R_TOP.pose_stamped.pose.position.y = 3.0; 
    CR_R_TOP.pose_stamped.pose.position.z = 0.0;
    CR_R_TOP.pose_stamped.pose.orientation.x = 0.0; 
    CR_R_TOP.pose_stamped.pose.orientation.y = 0.0; 
    CR_R_TOP.pose_stamped.pose.orientation.z = 0.0;
    CR_R_TOP.pose_stamped.pose.orientation.w = 0.0; 

    stored_goal_locations_.emplace_back(CR_R_TOP);

  LocationData CR_R_BOTTOM;
    CR_R_BOTTOM.name = "CR_R_BOTTOM";
    CR_R_BOTTOM.pose_stamped.header.frame_id = "map"; 
    CR_R_BOTTOM.pose_stamped.pose.position.x = 12.0; 
    CR_R_BOTTOM.pose_stamped.pose.position.y = -3.0; 
    CR_R_BOTTOM.pose_stamped.pose.position.z = 0.0;
    CR_R_BOTTOM.pose_stamped.pose.orientation.x = 0.0; 
    CR_R_BOTTOM.pose_stamped.pose.orientation.y = 0.0; 
    CR_R_BOTTOM.pose_stamped.pose.orientation.z = 0.0;
    CR_R_BOTTOM.pose_stamped.pose.orientation.w = 0.0; 

    stored_goal_locations_.emplace_back(CR_R_BOTTOM);

// Previous locations for the Plant testing map. They need to be updated with the correct orientation
// parameters in case they are used. 

  // LocationData dock_0;
  //   dock_0.name = "Dock 0";
  //   dock_0.pose_stamped.header.frame_id = "map"; 
  //   dock_0.pose_stamped.pose.position.x = 2.50; 
  //   dock_0.pose_stamped.pose.position.y = 4.00; 
  //   dock_0.pose_stamped.pose.position.z = 0.0;
  //   dock_0.pose_stamped.pose.orientation.w = 0.0; 

  //   stored_goal_locations_.emplace_back(dock_0);

  // LocationData dock_1;
  //   dock_1.name = "Dock 1";
  //   dock_1.pose_stamped.header.frame_id = "map"; 
  //   dock_1.pose_stamped.pose.position.x = 2.23; 
  //   dock_1.pose_stamped.pose.position.y = 1.73; 
  //   dock_1.pose_stamped.pose.position.z = 0.0;
  //   dock_1.pose_stamped.pose.orientation.w = 0.0;
  //   stored_goal_locations_.emplace_back(dock_1);

  // LocationData dock_2;
  //   dock_2.name = "Dock 2";
  //   dock_2.pose_stamped.header.frame_id = "map"; 
  //   dock_2.pose_stamped.pose.position.x = 1.81; 
  //   dock_2.pose_stamped.pose.position.y = -0.37; 
  //   dock_2.pose_stamped.pose.position.z = 0.0;
  //   dock_2.pose_stamped.pose.orientation.w = 0.0;
  //   stored_goal_locations_.emplace_back(dock_2);

  // LocationData payload_0;
  //   payload_0.name = "Payload 0";
  //   payload_0.pose_stamped.header.frame_id = "map"; 
  //   payload_0.pose_stamped.pose.position.x = 16.7; 
  //   payload_0.pose_stamped.pose.position.y = 14.78; 
  //   payload_0.pose_stamped.pose.position.z = 0.0;
  //   payload_0.pose_stamped.pose.orientation.w = 0.0;
  //   stored_goal_locations_.emplace_back(payload_0);

  // LocationData payload_1;
  //   payload_1.name = "Payload 1";
  //   payload_1.pose_stamped.header.frame_id = "map"; 
  //   payload_1.pose_stamped.pose.position.x = 8.79; 
  //   payload_1.pose_stamped.pose.position.y = 15.23; 
  //   payload_1.pose_stamped.pose.position.z = 0.0;
  //   payload_1.pose_stamped.pose.orientation.w = 0.0;
  //   stored_goal_locations_.emplace_back(payload_1);

  // LocationData storage_0;
  //   storage_0.name = "Storage 0";
  //   storage_0.pose_stamped.header.frame_id = "map"; 
  //   storage_0.pose_stamped.pose.position.x = 6.65; 
  //   storage_0.pose_stamped.pose.position.y = -0.8; 
  //   storage_0.pose_stamped.pose.position.z = 0.0;
  //   storage_0.pose_stamped.pose.orientation.w = 0.0;
  //   stored_goal_locations_.emplace_back(storage_0);

  // LocationData plane_deck;
  //   plane_deck.name = "Plane Deck";
  //   plane_deck.pose_stamped.header.frame_id = "map"; 
  //   plane_deck.pose_stamped.pose.position.x = 52.14; 
  //   plane_deck.pose_stamped.pose.position.y = 29.68; 
  //   plane_deck.pose_stamped.pose.position.z = 0.0;
  //   plane_deck.pose_stamped.pose.orientation.w = 0.0;
  //   stored_goal_locations_.emplace_back(plane_deck);

  populateComboBox(robot_goals_container_0_, stored_goal_locations_);
  populateComboBox(robot_goals_container_1_, stored_goal_locations_);
  populateComboBox(robot_goals_container_2_, stored_goal_locations_);

  go_to_goal_0_ = new QPushButton("Go to Goal", this);
  go_to_goal_1_ = new QPushButton("Go to Goal", this);
  go_to_goal_2_ = new QPushButton("Go to Goal", this);
  go_to_goal_all_ = new QPushButton("Execute all goals", this);

  connect(go_to_goal_0_, &QPushButton::released, this, &NeoFleetRViz2Plugin::pushButtonGoToGoalRobot0);
  connect(go_to_goal_1_, &QPushButton::released, this, &NeoFleetRViz2Plugin::pushButtonGoToGoalRobot1);
  connect(go_to_goal_2_, &QPushButton::released, this, &NeoFleetRViz2Plugin::pushButtonGoToGoalRobot2);
  connect(go_to_goal_all_, &QPushButton::released, this, &NeoFleetRViz2Plugin::pushButtonGoToGoalAllRobots);

  start_rviz_0_ = new QPushButton("RViz", this);
  start_rviz_1_ = new QPushButton("RViz", this);
  start_rviz_2_ = new QPushButton("RViz", this);

  robot0_name_ = new QLabel("Robot 0 - Select a Goal: ");
  robot1_name_ = new QLabel("Robot 1 - Select a Goal: ");
  robot2_name_ = new QLabel("Robot 2 - Select a Goal: ");

  // Replanning status information:
  QCheckBox* led_indicator_robot0 = new QCheckBox(this);
  led_indicator_robot0->setStyleSheet("QCheckBox::indicator { width: 20px; height: 20px; }" 
                              "QCheckBox::indicator:checked { background-color: green; }"
                              "QCheckBox::indicator:unchecked { background-color: red; }");
  led_indicator_robot0->setChecked(false);
  led_indicator_robot0->setEnabled(false);

  QCheckBox* led_indicator_robot1 = new QCheckBox(this);
  led_indicator_robot1->setStyleSheet("QCheckBox::indicator { width: 20px; height: 20px; }" 
                              "QCheckBox::indicator:checked { background-color: green; }"
                              "QCheckBox::indicator:unchecked { background-color: red; }");
  led_indicator_robot1->setChecked(false);
  led_indicator_robot1->setEnabled(false);

  QCheckBox* led_indicator_robot2 = new QCheckBox(this);
  led_indicator_robot2->setStyleSheet("QCheckBox::indicator { width: 20px; height: 20px; }" 
                              "QCheckBox::indicator:checked { background-color: green; }"
                              "QCheckBox::indicator:unchecked { background-color: red; }");
  led_indicator_robot2->setChecked(false);
  led_indicator_robot2->setEnabled(false);

  robot0_replanning_label = new QLabel("robot0 replanning status:");
  robot1_replanning_label = new QLabel("robot1 replanning status:");
  robot2_replanning_label = new QLabel("robot2 replanning status:");

  robot0_replanning_label->setAlignment(Qt::AlignRight);
  robot1_replanning_label->setAlignment(Qt::AlignRight);
  robot2_replanning_label->setAlignment(Qt::AlignRight);

  robot0_replanning_label->setMaximumWidth(200);
  robot1_replanning_label->setMaximumWidth(200);
  robot2_replanning_label->setMaximumWidth(200);

  vertical_line_0 = new QFrame();
  vertical_line_0->setFrameShape(QFrame::VLine);
  vertical_line_0->setFrameShadow(QFrame::Sunken);
  vertical_line_0->setLineWidth(1);

  vertical_line_1 = new QFrame();
  vertical_line_1->setFrameShape(QFrame::VLine);
  vertical_line_1->setFrameShadow(QFrame::Sunken);
  vertical_line_1->setLineWidth(1);

  vertical_line_2 = new QFrame();
  vertical_line_2->setFrameShape(QFrame::VLine);
  vertical_line_2->setFrameShadow(QFrame::Sunken);
  vertical_line_2->setLineWidth(1);

  connect(replanningChecker, &ReplanningStatusChecker::replanningStatusChangedRobot0, this, &NeoFleetRViz2Plugin::handleReplanningStatusChangedRobot0);
  connect(replanningChecker, &ReplanningStatusChecker::replanningStatusChangedRobot1, this, &NeoFleetRViz2Plugin::handleReplanningStatusChangedRobot1);
  connect(replanningChecker, &ReplanningStatusChecker::replanningStatusChangedRobot2, this, &NeoFleetRViz2Plugin::handleReplanningStatusChangedRobot2);
  
  // replanning_status_layout_ = new QVBoxLayout;
  // replanning_status_layout_->addWidget(new QLabel("Robot Replanning Status Panel:"));
  // replanning_status_layout_->addWidget(led_indicator_robot0);
  // replanning_status_layout_->addWidget(led_indicator_robot1);
  // replanning_status_layout_->addWidget(led_indicator_robot2);

  robot0_name_->setAlignment(Qt::AlignRight);
  robot1_name_->setAlignment(Qt::AlignRight);
  robot2_name_->setAlignment(Qt::AlignRight);

  robot0_layout_->addWidget(robot0_name_);
  robot1_layout_->addWidget(robot1_name_);
  robot2_layout_->addWidget(robot2_name_);

  robot0_layout_->addWidget(robot_goals_container_0_);
  robot1_layout_->addWidget(robot_goals_container_1_);
  robot2_layout_->addWidget(robot_goals_container_2_);

  robot0_layout_->addWidget(go_to_goal_0_);
  robot1_layout_->addWidget(go_to_goal_1_);
  robot2_layout_->addWidget(go_to_goal_2_);

  robot0_layout_->addWidget(start_rviz_0_);
  robot1_layout_->addWidget(start_rviz_1_);
  robot2_layout_->addWidget(start_rviz_2_);

  robot0_layout_->addWidget(vertical_line_0);
  robot1_layout_->addWidget(vertical_line_1);
  robot2_layout_->addWidget(vertical_line_2);

  robot0_layout_->addWidget(robot0_replanning_label);
  robot1_layout_->addWidget(robot1_replanning_label);
  robot2_layout_->addWidget(robot2_replanning_label);

  robot0_layout_->addWidget(led_indicator_robot0);
  robot1_layout_->addWidget(led_indicator_robot1);
  robot2_layout_->addWidget(led_indicator_robot2);

  robots_layout_->addWidget(new QLabel("Robot goal control panel:"));
  robots_layout_->addLayout(robot0_layout_);
  robots_layout_->addLayout(robot1_layout_);
  robots_layout_->addLayout(robot2_layout_);
  robots_layout_->addWidget(go_to_goal_all_);

  horizontal_line = new QFrame();
  horizontal_line->setFrameShape(QFrame::HLine);
  horizontal_line->setFrameShadow(QFrame::Sunken);
  horizontal_line->setLineWidth(1);
  
  // start_rviz_ = new QPushButton("RViz", this);
  robot_container_ = new QComboBox(this);
  robot_location_ = new QLabel(this);
  selected_robot_ = new QLabel(this);

  topic_layout_->addWidget(new QLabel("Select the target robot to interact on RVIZ:"), 1);
  topic_layout_->addWidget(robot_container_, 2);
  // topic_layout_->addWidget(start_rviz_);

  // Initialize the ptr as Null
  robot_ = NULL;

  // Lay out the topic field above the control widget.
  connect(
    robot_container_, QOverload<int>::of(&QComboBox::activated), this,
    &NeoFleetRViz2Plugin::setRobotName);
  // connect(start_rviz_, &QPushButton::released, this, &NeoFleetRViz2Plugin::launchRViz);

  connect(start_rviz_0_, &QPushButton::released, this, &NeoFleetRViz2Plugin::robot0_selector);
  connect(start_rviz_0_, &QPushButton::released, this, &NeoFleetRViz2Plugin::launchRViz);
  connect(start_rviz_1_, &QPushButton::released, this, &NeoFleetRViz2Plugin::robot1_selector);
  connect(start_rviz_1_, &QPushButton::released, this, &NeoFleetRViz2Plugin::launchRViz);
  connect(start_rviz_2_, &QPushButton::released, this, &NeoFleetRViz2Plugin::robot0_selector);
  connect(start_rviz_2_, &QPushButton::released, this, &NeoFleetRViz2Plugin::launchRViz);

  // Main Layout management
  
  top_layout_->addLayout(topic_layout_, 5);
  top_layout_->addWidget(warn_signal_, 1);
  top_layout_->addWidget(robot_location_, 1);
  top_layout_->addWidget(selected_robot_, 1);

  main_layout_->addLayout(top_layout_);
  main_layout_->addWidget(horizontal_line);
  main_layout_->addLayout(robots_layout_);

  // Full Layout management

  // full_layout = new QHBoxLayout;


  // full_layout->addLayout(main_layout_);
  // full_layout->addWidget(vertical_line);
  // full_layout->addLayout(replanning_status_layout_);

  setLayout(main_layout_);

  worker->moveToThread(thread);
  // connect(worker, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
  connect(thread, SIGNAL(started()), worker, SLOT(process()));
  connect(worker, &Worker::send_pos, this, &NeoFleetRViz2Plugin::update_pos);
  connect(worker, &Worker::send_goal, this, &NeoFleetRViz2Plugin::send_goal);
  connect(worker, SIGNAL(send_pos()), thread, SLOT(quit()));
  connect(worker, SIGNAL(send_pos()), worker, SLOT(deleteLater()));
  connect(thread, SIGNAL(send_pos()), thread, SLOT(deleteLater()));
  thread->start();
}

NeoFleetRViz2Plugin::~NeoFleetRViz2Plugin()
{
}

void Worker::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  goal_pose_ = pose;
  emit send_goal();
}

void Worker::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
{
  initial_pose_ = pose;
}


void NeoFleetRViz2Plugin::setRobotName()
{
  robot_name_ = robot_container_->currentText().toStdString();

  // Searching and assigning the corresponding pointers for selected robot
  auto search = worker->robot_identity_map_.find(robot_name_);

  if (search != worker->robot_identity_map_.end()) {
    robot_ = search->second;
    warn_signal_->clear();
  } else {
    RCLCPP_ERROR(
      worker->node_->get_logger(),
      "Robot not found in the drop down"
    );
    robot_ = NULL;
    warn_signal_->setText("Robot is not found in the list");
  }
}

void NeoFleetRViz2Plugin::send_goal()
{
  geometry_msgs::msg::PoseStamped pub_goal_pose;

  if (worker->goal_pose_) {
      pub_goal_pose = *worker->goal_pose_;
      auto check_action_server_ready =
        robot_->navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
      if (!check_action_server_ready) {
        RCLCPP_ERROR(
          worker->node_->get_logger(),
          "navigate_to_pose action server is not available."
        );
        return;
      }

      robot_->navigation_goal_.pose = pub_goal_pose;

      auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

      auto future_goal_handle =
        robot_->navigation_action_client_->async_send_goal(
        robot_->navigation_goal_,
        send_goal_options);

      worker->goal_pose_ = NULL;
    }
}

void NeoFleetRViz2Plugin::update_pos()
{
  if (!process_combo_) {
    for (int i = 0; i < worker->robot_namespaces_.size(); i++) {
      robot_list_.push_back(QString::fromStdString(worker->robot_namespaces_[i]));
    }
    robot_container_->addItems(robot_list_);
    process_combo_ = true;
  }

  if (robot_ == NULL) {
    return;
  }

  geometry_msgs::msg::TransformStamped robot_pose;

  try {
    // setting it to true, even if the robot is already localized
    robot_->is_localized_ = true;
    robot_pose = tf2_buffer_->lookupTransform(
      "map", robot_->robot_name_ + "base_footprint",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      client_node_->get_logger(),
      "Could not transform %s to %s: %s, check if initialpose published",
      "map", "base_footprint", ex.what());
    robot_->is_localized_ = false;
  }

  if (!robot_->is_localized_) {
    robot_location_->setText(
      QString::fromStdString("X: 0, Y: 0, Theta: 0 "));
    selected_robot_->setText(
      QString::fromStdString("Selected Robot: " + robot_->robot_name_));
  } else {
    selected_robot_->setText(
      "Selected Robot: " +
      QString::fromStdString(robot_->robot_name_));
    robot_location_->setText(
      "X: " + QString::number(robot_pose.transform.translation.x) +
      ", Y: " + QString::number(robot_pose.transform.translation.y) +
      ", Theta: " + QString::number(robot_pose.transform.rotation.z));
  }

  // Check for inital pose updates every cycle
  if (worker->initial_pose_) {
    // set it to true, if not set before
    std::cout << "Publishing initial pose to: " << robot_->robot_name_ << std::endl;
    robot_->is_localized_ = true;
    robot_->local_pos_pub_->publish(*worker->initial_pose_);
    worker->initial_pose_ = NULL;
  }
}

// void NeoFleetRViz2Plugin::launchRViz() //Old RViz used with the robot selector combobox
// {
//   std::string command =
//     "ros2 launch neo_nav2_bringup rviz_launch.py use_namespace:=True namespace:=";
//   command.append(robot_->robot_name_);
//   command.append("&");

//   system(command.c_str() );
// }

void NeoFleetRViz2Plugin::launchRViz()
{
  std::string command =
    "ros2 launch neo_nav2_bringup rviz_launch.py use_namespace:=True namespace:=";
  command.append(robot_goal_panel_selector);
  command.append("&");

  system(command.c_str() );
}


void NeoFleetRViz2Plugin::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void NeoFleetRViz2Plugin::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void NeoFleetRViz2Plugin::robot0_selector()
{
  robot_goal_panel_selector = robot_0;
}

void NeoFleetRViz2Plugin::robot1_selector()
{
  robot_goal_panel_selector = robot_1;
}

void NeoFleetRViz2Plugin::robot2_selector()
{
  robot_goal_panel_selector = robot_2;
}

void NeoFleetRViz2Plugin::populateComboBox(QComboBox* comboBox, const std::vector<LocationData>& locations) {
  for (const auto& location : locations) {
      comboBox->addItem(QString::fromStdString(location.name));
  }
}

void NeoFleetRViz2Plugin::pushButtonGoToGoalRobot0(){
  nav2GoalCaller->publishGoal("robot0", comboboxLocationSelector(robot_goals_container_0_));
}

void NeoFleetRViz2Plugin::pushButtonGoToGoalRobot1(){
  nav2GoalCaller->publishGoal("robot1", comboboxLocationSelector(robot_goals_container_1_));
}

void NeoFleetRViz2Plugin::pushButtonGoToGoalRobot2(){
  nav2GoalCaller->publishGoal("robot2", comboboxLocationSelector(robot_goals_container_2_));
}

void NeoFleetRViz2Plugin::pushButtonGoToGoalAllRobots(){
  nav2GoalCaller->publishGoal("robot0", comboboxLocationSelector(robot_goals_container_0_));
  nav2GoalCaller->publishGoal("robot1", comboboxLocationSelector(robot_goals_container_1_));
  nav2GoalCaller->publishGoal("robot2", comboboxLocationSelector(robot_goals_container_2_));
}

void NeoFleetRViz2Plugin::handleReplanningStatusChangedRobot0(bool status){
  if (status==true)
  {
    led_indicator_robot0->setChecked(true);
  }
  else
  {
    led_indicator_robot0->setChecked(false);
  }
}

void NeoFleetRViz2Plugin::handleReplanningStatusChangedRobot1(bool status){
  if (status==true)
  {
    led_indicator_robot1->setChecked(true);
  }
  else
  {
    led_indicator_robot1->setChecked(false);
  }
}

void NeoFleetRViz2Plugin::handleReplanningStatusChangedRobot2(bool status){
  if (status==true)
  {
    led_indicator_robot2->setChecked(true);
  }
  else
  {
    led_indicator_robot2->setChecked(false);
  }
}

LocationData NeoFleetRViz2Plugin::comboboxLocationSelector(QComboBox * combobox){
  LocationData location_to_return;
  if(combobox->currentIndex() >= 0){
    std::string selected_location_name = combobox->currentText().toStdString();
    for (const auto& location : stored_goal_locations_)
      if(location.name == selected_location_name){
          location_to_return = location;
          return location_to_return;
          // break;
        }
    }
  return location_to_return; //Add a better method handle for the return case with an empty location case
}

// Nav2 Goal Publisher class

Nav2GoalPublisher::Nav2GoalPublisher() : Node("nav2_goal_publisher") {
    publisher_robot_0 = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot0/goal_pose", 10);
    publisher_robot_1 = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot1/goal_pose", 10);
    publisher_robot_2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot2/goal_pose", 10);
}

Nav2GoalPublisher::~Nav2GoalPublisher()
{
}

void Nav2GoalPublisher::publishGoal(std::string robot_name_selection, LocationData goal_info) {
    geometry_msgs::msg::PoseStamped goal;
    // Set frame ID and timestamp
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();

    goal.pose.position.x = goal_info.pose_stamped.pose.position.x;
    goal.pose.position.y = goal_info.pose_stamped.pose.position.y;
    goal.pose.position.z = goal_info.pose_stamped.pose.position.z;
    goal.pose.orientation.x = goal_info.pose_stamped.pose.orientation.x;
    goal.pose.orientation.y = goal_info.pose_stamped.pose.orientation.y;
    goal.pose.orientation.z = goal_info.pose_stamped.pose.orientation.z;
    goal.pose.orientation.w = goal_info.pose_stamped.pose.orientation.w;

    // Publish the goal once
    if(robot_name_selection == "robot0")
    {
      publisher_robot_0->publish(goal);
    }
    else if(robot_name_selection == "robot1")
    {
      publisher_robot_1->publish(goal);
    }
    else if(robot_name_selection == "robot2")
    {
      publisher_robot_2->publish(goal);
    }
    RCLCPP_INFO(this->get_logger(), "Published navigation goal");
}


ReplanningStatusChecker::ReplanningStatusChecker() : Node("replanning_status_checker") 
{
  replanning_status_checker_node_ = std::make_shared<rclcpp::Node>("__");

  robot0_replanning_status_sub_ = replanning_status_checker_node_->create_subscription<std_msgs::msg::Bool>(
    "robot0/robot_replanning_status_topic", 1, std::bind(&ReplanningStatusChecker::robot0_replanning_status_callback, this, std::placeholders::_1));
  robot1_replanning_status_sub_ = replanning_status_checker_node_->create_subscription<std_msgs::msg::Bool>(
    "robot1/robot_replanning_status_topic", 1, std::bind(&ReplanningStatusChecker::robot1_replanning_status_callback, this, std::placeholders::_1));
  robot2_replanning_status_sub_ = replanning_status_checker_node_->create_subscription<std_msgs::msg::Bool>(
    "robot2/robot_replanning_status_topic", 1, std::bind(&ReplanningStatusChecker::robot2_replanning_status_callback, this, std::placeholders::_1));  

}

ReplanningStatusChecker::~ReplanningStatusChecker()
{
}


void ReplanningStatusChecker::robot0_replanning_status_callback(const std_msgs::msg::Bool::SharedPtr robot0_replanning_status)
{
  robot0_replanning_status_ = robot0_replanning_status;
  emit replanningStatusChangedRobot0(robot0_replanning_status_->data);
}

void ReplanningStatusChecker::robot1_replanning_status_callback(const std_msgs::msg::Bool::SharedPtr robot1_replanning_status)
{
  robot1_replanning_status_ = robot1_replanning_status;
  emit replanningStatusChangedRobot1(robot1_replanning_status_->data);
}

void ReplanningStatusChecker::robot2_replanning_status_callback(const std_msgs::msg::Bool::SharedPtr robot2_replanning_status)
{
  robot2_replanning_status_ = robot2_replanning_status;
  emit replanningStatusChangedRobot2(robot2_replanning_status_->data);
}


}  // namespace neo_fleet

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(neo_fleet::NeoFleetRViz2Plugin, rviz_common::Panel)
