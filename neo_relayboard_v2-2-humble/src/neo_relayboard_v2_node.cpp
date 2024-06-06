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

#include "../common/include/NeoRelayBoardNode.h"
#include <chrono>

// //#######################
// //#### main programm ####
int main(int argc, char **argv)
{
	// initialize ROS
	rclcpp::init(argc, argv);
	auto nh = std::make_shared<NeoRelayBoardNode>();
	
	// get parameters
	double request_rate;   // [1/s]
	nh->declare_parameter<double>("request_rate");
	nh->get_parameter_or("request_rate", request_rate, 25.0);

	// frequency of publishing states (cycle time)
	rclcpp::Rate loop_rate(request_rate);

	if (nh->init() != 0)
			return 1;

	while (rclcpp::ok())
	{
		// initialize node
		
		const rclcpp::Time cycleStartTime = rclcpp::Clock().now();

		// Communication
		const int comState = nh->HandleCommunication();

		// RelayBoard
		nh->PublishRelayBoardState();
		nh->PublishBatteryState();
		nh->PublishEmergencyStopStates();

		// Motors
		nh->PublishJointStates();

		// IOBoard
		nh->PublishIOBoard();

		// USBoard
		nh->PublishUSBoardData();

		rclcpp::spin_some(nh);

		const rclcpp::Duration cycleTime = rclcpp::Clock().now() - cycleStartTime;

		// RCLCPP_INFO(nh->get_logger(), "cycleTime: %f", cycleTime.seconds());

		// Check if to restart node in case of error
		bool restart = false;
		switch(comState)
		{
		case neo_msgs2::msg::RelayBoardV2::CS_OK:
		case neo_msgs2::msg::RelayBoardV2::CS_CONFIGURATION_FAILED:
			break;
		default:
			if(rclcpp::ok())
			{
				RCLCPP_WARN(nh->get_logger(), "Communication error, restarting node ...");
			    std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			restart = true;
		}

		if(restart) {
			RCLCPP_WARN(nh->get_logger(), "Communication error, trying restarting node ...");
			restart = false;
		}
		loop_rate.sleep();
	}


	return 0;
}
