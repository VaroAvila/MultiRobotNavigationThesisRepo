/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
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

#include "../include/SickS300Receiver.h"

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/shared_ptr.hpp>


class SickS300ReceiverROS : public SickS300Receiver, public rclcpp::Node 
{
public:
	double scan_cycle_time = 0;
	double scan_duration = 0;
	double scan_delay = 0;
	double angle_min = -135.0/180.0 * M_PI;
	double angle_max = 135.0/180.0 * M_PI;
	double range_min = 0.01;
	double range_max = 29.0; // 30m is maximum, report less to avoid confusion between true 30m and infinity
	std::string frame_id;

	SickS300ReceiverROS(): Node("neo_sick_s300_node")
	{
		this->declare_parameter<double>("scan_cycle_time", 0.040);
		this->declare_parameter<double>("scan_duration", 0.025); 
		this->declare_parameter<double>("scan_delay", 0.045);				
		this->declare_parameter<std::string>("frame_id", "lidar_1_link");

		this->get_parameter("scan_cycle_time", scan_cycle_time);
		this->get_parameter("scan_duration", scan_duration); // scan_cycle_time * 270/360
		this->get_parameter("scan_delay", scan_delay);			// 20 ms transmission + 10 ms processing
		this->get_parameter("frame_id", frame_id);

		m_topic_scan = this->create_publisher<sensor_msgs::msg::LaserScan>("scan",1); 
	}

protected:
	void handle_scan(const std::vector<point_t>& points) override
	{
		sensor_msgs::msg::LaserScan msg;

		msg.header.stamp = rclcpp::Clock().now() - rclcpp::Duration::from_seconds(scan_duration) - 
			rclcpp::Duration::from_seconds(scan_delay);

		// Handle namespaces
		std::string robot_namespace(this->get_namespace());
		
		// removing the unnecessary "/" from the namespace
		robot_namespace.erase(0, 1); // The frame id would be lidar_1_link / robot1/lidar_1_link

		msg.header.frame_id = robot_namespace + "_link";
		const size_t num_points = points.size();
		msg.angle_min = angle_min;
		msg.angle_max = angle_max;
		msg.range_min = range_min;
		msg.range_max = range_max;
		msg.angle_increment = (angle_max - angle_min) / (num_points - 1);
		msg.time_increment = scan_duration / (num_points - 1);
		msg.scan_time = scan_cycle_time;

		msg.ranges.resize(num_points);
		msg.intensities.resize(num_points);
		for(size_t i = 0; i < num_points; ++i)
		{
			msg.ranges[i] = points[i].distance;
			msg.intensities[i] = points[i].reflector ? 1.f : 0.f;
		}

		m_topic_scan->publish(msg);
	}

	void handle_debug_msg(const std::string& msg) override
	{
		RCLCPP_INFO(this->get_logger(),"%s", msg.c_str());
	}

private:
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_topic_scan;

};


//#######################
//#### main programm ####
int main(int argc, char **argv)
{
	// initialize ROS
	rclcpp::init(argc, argv);

	// keep a node handle outside the loop to prevent auto-shutdown
	auto nh = std::make_shared<SickS300ReceiverROS>();

	std::string port;
	int baud_rate = 0;

	nh->declare_parameter<std::string>("port", "/dev/neo-s300-1");
	nh->declare_parameter<int>("baud", 500000);
	nh->get_parameter("port", port);
	nh->get_parameter("baud", baud_rate);

	while(rclcpp::ok())
	{
		try {
			nh->open(port, baud_rate);
		}
		catch(std::exception& ex)
		{
			RCLCPP_ERROR(nh->get_logger(), "Failed to open S300 port '%s': %s",  port.c_str(), ex.what());
			std::this_thread::sleep_for(std::chrono::seconds(1));
			continue;
		}

		RCLCPP_INFO(nh->get_logger(), "Opened S300 port %s", port.c_str());

		while(rclcpp::ok())
		{
			try {
				nh->read(1000);
			}
			catch(std::exception& ex)
			{
				if(rclcpp::ok())
				{
					RCLCPP_ERROR(nh->get_logger(), "S300 port '%s' read error: %s", port.c_str(), ex.what());
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}
				break;
			}
		}
	}

	return 0;
}
