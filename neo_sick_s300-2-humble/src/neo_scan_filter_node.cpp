/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_sick_s300
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Jan 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
#include <vector>
#include <algorithm>

// ROS includes
#include <rclcpp/rclcpp.hpp>

// ROS message includes
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

//####################
//#### node class ####
class NodeClass : public rclcpp::Node 
{
public:
	std::vector<std::vector<double>> scan_intervals;

	// topics to publish and subscribe
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr topicPub_laser_scan;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr topicSub_laser_scan_raw;

	NodeClass(): Node("neo_scan_filter_node")
	{
		// loading config
		this->declare_parameter< std::vector<double> >("scan_intervals", {-2.27, 2.27});

		scan_intervals = loadScanRanges();

		// implementation of topics to publish
		topicPub_laser_scan = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 1);
		topicSub_laser_scan_raw = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&NodeClass::scanCallback, this,_1));
	}

	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
		// use hole received message, later only clear some ranges
		sensor_msgs::msg::LaserScan laser_scan;

		laser_scan = *msg;

		//if no filter intervals specified
		if (scan_intervals.size() == 0)
		{
			topicPub_laser_scan->publish(laser_scan);
			return;
		}

		int start_scan, stop_scan, num_scans;
		num_scans = laser_scan.ranges.size();

		stop_scan = 0;

		for (unsigned int i = 0; i < scan_intervals.size(); i++)
		{
			std::vector<double> *it = &scan_intervals.at(i);

			if (it->at(1) <= laser_scan.angle_min)
			{
				RCLCPP_WARN(this->get_logger(),"Found an interval that lies below min scan range, skip!");
				continue;
			}
			if (it->at(0) >= laser_scan.angle_max)
			{
				RCLCPP_WARN(this->get_logger(),"Found an interval that lies beyond max scan range, skip!");
				continue;
			}

			if (it->at(0) <= laser_scan.angle_min)
				start_scan = 0;
			else
			{
				start_scan = (int)((it->at(0) - laser_scan.angle_min) / laser_scan.angle_increment);
			}

			for (int u = stop_scan; u < start_scan; u++)
			{
				laser_scan.ranges.at(u) = 0.0; //laser_scan.range_min;
			}

			if (it->at(1) >= laser_scan.angle_max)
				stop_scan = num_scans - 1;
			else
			{
				stop_scan = (int)((it->at(1) - laser_scan.angle_min) / laser_scan.angle_increment);
			}
		}

		for (unsigned int u = stop_scan; u < laser_scan.ranges.size(); u++)
		{
			laser_scan.ranges.at(u) = 0.0; //laser_scan.range_min;
		}

		// publish message
		topicPub_laser_scan->publish(laser_scan);
	}

	std::vector<std::vector<double>> loadScanRanges()
	{
		std::string scan_intervals_param = "scan_intervals";
		std::vector<std::vector<double>> vd_interval_set;
		std::vector<double> vd_interval;
		std::vector<double> intervals_list;

		//grab the range-list from the parameter server if possible
		this->get_parameter("scan_intervals",vd_interval);
		intervals_list = vd_interval;


		//make sure we have a list of lists
			if (!(intervals_list.size() == 2))
			{
				RCLCPP_ERROR(this->get_logger(),"The scan intervals must be specified as a list of lists [[x1, y1], [x2, y2], ..., [xn, yn]]");
				throw std::runtime_error("The scan intervals must be specified as a list of lists [[x1, y1], [x2, y2], ..., [xn, yn]]");
			}

			for (int i = 0; i < (int)intervals_list.size(); ++i)
			{
				vd_interval.clear();

				auto interval = intervals_list[i];

				//basic checking validity
				if (intervals_list.at(0) > M_PI || intervals_list.at(1) > M_PI)
				{
					RCLCPP_WARN(this->get_logger(),"Found a scan interval > PI, skip!");
					continue;
					//throw std::runtime_error("Found a scan interval > PI!");
				}

				if (intervals_list.at(0) >= intervals_list.at(1))
				{
					RCLCPP_WARN(this->get_logger(),"Found a scan interval with i1 > i2, switched order!");
					intervals_list[1] = intervals_list[0];
					intervals_list[0] = (typeid(interval) == typeid(int) ? (int)(interval) : (double)(interval));
				}

				vd_interval_set.push_back(intervals_list);
			}
	
		//now we want to sort the intervals and check for overlapping
		// sort(vd_interval_set.begin(), vd_interval_set.end(), compareIntervals);

		// for (unsigned int i = 0; i < vd_interval_set.size(); i++)
		// {
		// 	for (unsigned int u = i + 1; u < vd_interval_set.size(); u++)
		// 	{
		// 		if (vd_interval_set.at(i).at(1) > vd_interval_set.at(u).at(0))
		// 		{
		// 			RCLCPP_ERROR(this->get_logger(),"The scan intervals you specified are overlapping!");
		// 			throw std::runtime_error("The scan intervals you specified are overlapping!");
		// 		}
		// 	}
		// }

		/* DEBUG out:
		for(unsigned int i = 0; i<vd_interval_set.size(); i++) {
			std::cout << "Interval " << i << " is " << vd_interval_set.at(i).at(0) << " | " << vd_interval_set.at(i).at(1) << std::endl;
		} */

		return vd_interval_set;
	}

static bool compareIntervals(std::vector<double> a, std::vector<double> b)
{
	return a.at(0) < b.at(0);
}

};

//#######################
//#### main programm ####
int main(int argc, char **argv)
{
	// initialize ROS, specify name of node
	rclcpp::init(argc, argv);

	auto nh = std::make_shared<NodeClass>();

	rclcpp::spin(nh);

	return 0;
}



