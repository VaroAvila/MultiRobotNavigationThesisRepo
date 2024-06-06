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

// basics
#include <sstream>
#include <iostream>

// ROS includes
#include "rclcpp/rclcpp.hpp"

// ROS message includes
#include <unistd.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "neo_msgs2/msg/emergency_stop_state.hpp"
#include "neo_msgs2/msg/us_board.hpp"
#include "neo_msgs2/msg/relay_board_v2.hpp"
#include "neo_msgs2/msg/io_board.hpp"

// ROS service includes
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <neo_srvs2/srv/io_board_set_dig_out.hpp>
#include <neo_srvs2/srv/relay_board_set_relay.hpp>
#include <neo_srvs2/srv/relay_board_set_lcd_msg.hpp>
#include <neo_srvs2/srv/relay_board_set_em_stop.hpp>
#include <neo_srvs2/srv/relay_board_un_set_em_stop.hpp>

#include "RelayBoardClient.h"


//####################
//#### node class ####
class NeoRelayBoardNode :public rclcpp::Node 
{
public:
	NeoRelayBoardNode();

	~NeoRelayBoardNode();


	// ---- Comm Handler ----
	int init();
	int HandleCommunication();

	// ---- Topic Callbacks ----
	void getNewVelocitiesFomTopic(const trajectory_msgs::msg::JointTrajectory::SharedPtr jt);

	// ---- Pubisher functions ----
	// RelayBoard
	void PublishRelayBoardState();
	void PublishBatteryState();
	void PublishEmergencyStopStates();
	// Motors
	void PublishJointStates();
	// USBoard
	void PublishUSBoardData();
	// IOBoard
	void PublishIOBoard();

	// ---- Service Callbacks ----
	bool serviceRelayBoardSetEmStop(std::shared_ptr<neo_srvs2::srv::RelayBoardSetEMStop::Request>, std::shared_ptr<neo_srvs2::srv::RelayBoardSetEMStop::Response> res);
	bool serviceRelayBoardUnSetEmStop(std::shared_ptr<neo_srvs2::srv::RelayBoardUnSetEMStop::Request>, std::shared_ptr<neo_srvs2::srv::RelayBoardUnSetEMStop::Response> res);
	bool serviceRelayBoardSetRelay(std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request>, std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Response> res);
	bool serviceRelayBoardSetLCDMsg(std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Request>, std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Response> res);
	bool serviceStartCharging(std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response> res);
	bool serviceStopCharging(std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response> res);
	bool serviceIOBoardSetDigOut(std::shared_ptr<neo_srvs2::srv::IOBoardSetDigOut::Request>, std::shared_ptr<neo_srvs2::srv::IOBoardSetDigOut::Response> res);

private:

	// basic topics:

	rclcpp::Publisher<neo_msgs2::msg::RelayBoardV2>::SharedPtr topicPub_RelayBoardState;
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr topicPub_BatteryState;
	rclcpp::Publisher<neo_msgs2::msg::EmergencyStopState>::SharedPtr topicPub_isEmergencyStop;
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr topicPub_batVoltage;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topicPub_drives;
	rclcpp::Publisher<neo_msgs2::msg::USBoard>::SharedPtr topicPub_usBoard;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr topicPub_USRangeSensor[16];
	rclcpp::Publisher<neo_msgs2::msg::IOBoard>::SharedPtr topicPub_IOBoard;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_StartCharging;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_StopCharging;
	rclcpp::Service<neo_srvs2::srv::RelayBoardSetRelay>::SharedPtr srv_SetRelay;
	rclcpp::Service<neo_srvs2::srv::RelayBoardSetLCDMsg>::SharedPtr srv_SetLCDMsg;
	rclcpp::Service<neo_srvs2::srv::RelayBoardSetEMStop>::SharedPtr srv_SetEMStop;
	rclcpp::Service<neo_srvs2::srv::RelayBoardUnSetEMStop>::SharedPtr srv_UnSetEMStop;
	rclcpp::Service<neo_srvs2::srv::IOBoardSetDigOut>::SharedPtr srv_SetDigOut;

	// Drives:
	rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr topicSub_drives;



	// ---- Communication --------
	int m_iComState = neo_msgs2::msg::RelayBoardV2::CS_NOT_ESTABLISHED;

	// ---- Configuration --------
	int m_iactive_motors = 0;
	int m_imotor_count = 0;
	int m_ihoming_motors = 0;
	int m_iext_hardware = 0;

	// true if RelayBoard has confirmed the configuration and is ready
	bool m_bRelayBoardV2Available = false;

	// ---- Battery ------------
	std::string m_sBatterySerialNumber = "NeoBattery999";
	std::string m_sBatteryLocation = "Slot999";
	int m_iBatteryChemistry = 1;
	float m_fBatteryDesignCapacity = 100.f;

	// ---- IOBoard ------------
	bool m_bIOBoardActive = false;

	// ---- USBoard ------------
	bool m_bUSBoardActive = false;
	bool m_bUSBoardSensorActive[16] = {};

	std::string m_sComPort;
	RelayBoardClient *m_SerRelayBoard = 0;

	// ---- Motors ----
	DriveParam m_Drives[8];

	// ---- EM Stop Handling ------
	int m_iEM_stop_state = ST_EM_FREE;
	double m_duration_for_EM_free = 1.0;
	rclcpp::Time m_time_of_EM_confirmed;

	// possible states of emergency stop
	enum
	{
		ST_EM_FREE = 0,
		ST_EM_ACTIVE = 1,
		ST_EM_CONFIRMED = 2
	};

	// ---- Msg Handling ------------
	int m_iLastRXReturn = -1;
	rclcpp::Time m_tCurrentTimeStamp;
	rclcpp::Time m_last_trajectory_time;
	double m_tMotorDelay = 0;
	double m_trajectory_timeout = 0;
	bool is_trajectory_timeout = false;
	bool m_JointStates;
	int m_drivesNr;
	bool m_bSoftware_EM_stop = false;

	// log
	bool m_bLog = false;	// enables or disables the log for neo_relayboard

};
