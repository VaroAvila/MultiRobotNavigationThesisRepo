/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Neobotix GmbH
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
 *     with the distributiothis->
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permissiothis->
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

#include <math.h>
#include "rclcpp/rclcpp.hpp"

#include "../include/NeoRelayBoardNode.h"

using std::placeholders::_1;
using std::placeholders::_2;


NeoRelayBoardNode::NeoRelayBoardNode(): Node("neo_relayboard_node")
{
	// Declare Parameters for getting the data from the parameter server

    this->declare_parameter<std::string>("port", "random");
    this->declare_parameter<std::string>("battery.serial_number", "random");
    this->declare_parameter<std::string>("battery.location", "random");
    this->declare_parameter<float>("battery.design_capacity", 2.0);
    this->declare_parameter<int>("battery.chemistry", 3);
    this->declare_parameter<bool>("log", 0);
    this->declare_parameter<bool>("Publish_joint_states", 0);
    this->declare_parameter<bool>("ioboard.active", 0);
	this->declare_parameter<bool>("usboard.active", 0);
	this->declare_parameter<bool>("usboard.sensor1_active", 0);
	this->declare_parameter<bool>("usboard.sensor2_active", 0);
	this->declare_parameter<bool>("usboard.sensor3_active", 0);
	this->declare_parameter<bool>("usboard.sensor4_active", 0);
	this->declare_parameter<bool>("usboard.sensor5_active", 0);
	this->declare_parameter<bool>("usboard.sensor6_active", 0);
	this->declare_parameter<bool>("usboard.sensor7_active", 0);
	this->declare_parameter<bool>("usboard.sensor8_active", 0);
	this->declare_parameter<bool>("usboard.sensor9_active", 0);
	this->declare_parameter<bool>("usboard.sensor10_active",0);
	this->declare_parameter<bool>("usboard.sensor11_active",0);
	this->declare_parameter<bool>("usboard.sensor12_active",0);
	this->declare_parameter<bool>("usboard.sensor13_active",0);
	this->declare_parameter<bool>("usboard.sensor14_active",0);
	this->declare_parameter<bool>("usboard.sensor15_active",0);
	this->declare_parameter<bool>("usboard.sensor16_active",0);
	this->declare_parameter<int>("number_of_drives", 8);
	this->declare_parameter<double>("motor_delay", 0.0);
	this->declare_parameter<double>("trajectory_timeout", 0.0);
}

NeoRelayBoardNode::~NeoRelayBoardNode()
{
	delete m_SerRelayBoard;
}

int NeoRelayBoardNode::init()
{
	// Relayboard Config Parameter

	if (this->has_parameter("port"))
	{
		this->get_parameter("port", m_sComPort);
		RCLCPP_INFO(this->get_logger(), "Loaded ComPort parameter from parameter server: %s", m_sComPort.c_str());
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(),"FAILED to load ComPort parameter from parameter server");
		return 1;
	}

	std::cout << "                                                                     "<<std::endl;
	std::cout << "    NN    N  EEEEE   OOOOO   BBBBB    OOOOO   TTTTTTT  I  X   X      "<<std::endl;
	std::cout << "    N N   N  E      O     O  B    B  O     O     T     I   X X       "<<std::endl;
	std::cout << "    N  N  N  EEEEE  O     O  BBBBB   O     O     T     I    X        "<<std::endl;
	std::cout << "    N   N N  E      O     O  B    B  O     O     T     I   X X       "<<std::endl;
	std::cout << "    N    NN  EEEEE   OOOOO   BBBBB    OOOOO      T     I  X   X      "<<std::endl;
	std::cout << "                                                                     "<<std::endl;
	
	//---------------------------------------- GET PARAMS -----------------------------------------------------------
	this->get_parameter_or("motor_delay", m_tMotorDelay, 0.0);
	this->get_parameter_or("trajectory_timeout", m_trajectory_timeout, 1.);
	this->get_parameter_or("number_of_drives", m_drivesNr, 8);

	// Battery

	this->get_parameter("battery.serial_number", m_sBatterySerialNumber);
	this->get_parameter("battery.location", m_sBatteryLocation);
	this->get_parameter("battery.design_capacity", m_fBatteryDesignCapacity);
	this->get_parameter("battery.chemistry", m_iBatteryChemistry);

	// Logging
	this->get_parameter("log", m_bLog);

	// IOBOard Parameter
	this->get_parameter("ioboard.active", m_bIOBoardActive);

	// Joint states
	this->get_parameter("Publish_joint_states", m_JointStates);

	// USBOard Parameter
	this->get_parameter("usboard.active", m_bUSBoardActive);
	this->get_parameter("usboard.sensor1_active", m_bUSBoardSensorActive[0]);
	this->get_parameter("usboard.sensor2_active", m_bUSBoardSensorActive[1]);
	this->get_parameter("usboard.sensor3_active", m_bUSBoardSensorActive[2]);
	this->get_parameter("usboard.sensor4_active", m_bUSBoardSensorActive[3]);
	this->get_parameter("usboard.sensor5_active", m_bUSBoardSensorActive[4]);
	this->get_parameter("usboard.sensor6_active", m_bUSBoardSensorActive[5]);
	this->get_parameter("usboard.sensor7_active", m_bUSBoardSensorActive[6]);
	this->get_parameter("usboard.sensor8_active", m_bUSBoardSensorActive[7]);
	this->get_parameter("usboard.sensor9_active", m_bUSBoardSensorActive[8]);
	this->get_parameter("usboard.sensor10_active", m_bUSBoardSensorActive[9]);
	this->get_parameter("usboard.sensor11_active", m_bUSBoardSensorActive[10]);
	this->get_parameter("usboard.sensor12_active", m_bUSBoardSensorActive[11]);
	this->get_parameter("usboard.sensor13_active", m_bUSBoardSensorActive[12]);
	this->get_parameter("usboard.sensor14_active", m_bUSBoardSensorActive[13]);
	this->get_parameter("usboard.sensor15_active", m_bUSBoardSensorActive[14]);
	this->get_parameter("usboard.sensor16_active", m_bUSBoardSensorActive[15]);

	// Motor Parameter
	for (int i = 0; i < m_drivesNr; ++i)
	{
		const std::string parent = "drive" + std::to_string(i + 2) + ".";

		// Declaring parameters
		this->declare_parameter<bool>(parent + "motor_active", false);
		this->declare_parameter<bool>(parent + "homing_active", false);
		this->declare_parameter<int>(parent + "EncIncrPerRevMot", 0);
		this->declare_parameter<double>(parent + "VelMeasFrqHz", 0.0);
		this->declare_parameter<double>(parent + "GearRatio", 0.0);
		this->declare_parameter<double>(parent + "BeltRatio", 0.0);
		this->declare_parameter<int>(parent + "Sign", 0);
		this->declare_parameter<double>(parent + "VelMaxEncIncrS", 0.0);
		this->declare_parameter<double>(parent + "VelPModeEncIncrS", 0.0);
		this->declare_parameter<double>(parent + "AccIncrS2", 0.0);
		this->declare_parameter<double>(parent + "DecIncrS2", 0.0);
		this->declare_parameter<double>(parent + "Modulo", 0.0);
		this->declare_parameter<std::string>(parent+ "joint_name", "Joint999");
		m_Drives[i].sName = "Joint999";

		// getting parameters
		this->get_parameter(parent + "motor_active", m_Drives[i].bmotor_active);
		this->get_parameter(parent + "homing_active", m_Drives[i].bhoming_active);
		this->get_parameter(parent + "EncIncrPerRevMot", m_Drives[i].iEncIncrPerRevMot);
		this->get_parameter(parent + "VelMeasFrqHz", m_Drives[i].dVelMeasFrqHz);
		this->get_parameter(parent + "GearRatio", m_Drives[i].dGearRatio);
		this->get_parameter(parent + "BeltRatio", m_Drives[i].dBeltRatio);
		this->get_parameter(parent + "Sign", m_Drives[i].iSign);
		this->get_parameter(parent + "VelMaxEncIncrS", m_Drives[i].dVelMaxEncIncrS);
		this->get_parameter(parent + "VelPModeEncIncrS", m_Drives[i].dVelPModeEncIncrS);
		this->get_parameter(parent + "AccIncrS2", m_Drives[i].dAccIncrS2);
		this->get_parameter(parent + "DecIncrS2", m_Drives[i].dDecIncrS2);
		this->get_parameter(parent + "Modulo", m_Drives[i].dModulo);
		this->get_parameter(parent + "joint_name", m_Drives[i].sName);

		m_Drives[i].calcRadToIncr();
	}

	// Check which motors are active
	if (m_Drives[0].bmotor_active)
	{
		m_iactive_motors += 1;
		m_imotor_count++;
	}
	if (m_Drives[1].bmotor_active)
	{
		m_iactive_motors += 2;
		m_imotor_count++;
	}
	if (m_Drives[2].bmotor_active)
	{
		m_iactive_motors += 4;
		m_imotor_count++;
	}
	if (m_Drives[3].bmotor_active)
	{
		m_iactive_motors += 8;
		m_imotor_count++;
	}
	if (m_Drives[4].bmotor_active)
	{
		m_iactive_motors += 16;
		m_imotor_count++;
	}
	if (m_Drives[5].bmotor_active)
	{
		m_iactive_motors += 32;
		m_imotor_count++;
	}
	if (m_Drives[6].bmotor_active)
	{
		m_iactive_motors += 64;
		m_imotor_count++;
	}
	if (m_Drives[7].bmotor_active)
	{
		m_iactive_motors += 128;
		m_imotor_count++;
	}

	// Check if homing is active
	if (m_Drives[0].bhoming_active)
		m_ihoming_motors += 1;
	if (m_Drives[1].bhoming_active)
		m_ihoming_motors += 2;
	if (m_Drives[2].bhoming_active)
		m_ihoming_motors += 4;
	if (m_Drives[3].bhoming_active)
		m_ihoming_motors += 8;
	if (m_Drives[4].bhoming_active)
		m_ihoming_motors += 16;
	if (m_Drives[5].bhoming_active)
		m_ihoming_motors += 32;
	if (m_Drives[6].bhoming_active)
		m_ihoming_motors += 64;
	if (m_Drives[7].bhoming_active)
		m_ihoming_motors += 128;

	// Check external hardware
	if (m_bIOBoardActive)
		m_iext_hardware += 1;
	if (m_bUSBoardActive)
		m_iext_hardware += 2;

	RCLCPP_INFO(this->get_logger(),"Parameters loaded");
	//----------------------------------------OPEN COMPORT---------------------------------------------------------

	m_SerRelayBoard = new RelayBoardClient();

	// Logging yes/no
	if (m_bLog)
	{
		RCLCPP_INFO(this->get_logger(),"Log: enabled");
		m_SerRelayBoard->enable_logging();
	}
	else
	{
		RCLCPP_INFO(this->get_logger(),"Log: disabled");
		m_SerRelayBoard->disable_logging();
	}

	// Initialize SerialBoard
	const int ret = m_SerRelayBoard->init(
		m_sComPort.c_str(), m_iactive_motors, m_ihoming_motors, m_iext_hardware,
		m_Drives[0].dModulo, m_Drives[1].dModulo, m_Drives[2].dModulo, m_Drives[3].dModulo,
		m_Drives[4].dModulo, m_Drives[5].dModulo, m_Drives[6].dModulo, m_Drives[7].dModulo);

	if (ret == RelayBoardClient::INIT_CONFIG_OK)
	{
		m_bRelayBoardV2Available = true;
		m_iComState = neo_msgs2::msg::RelayBoardV2::CS_OK;
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(),"FAILED to open RelayboardV2 at ComPort %s", m_sComPort.c_str());
		m_bRelayBoardV2Available = false;

		switch(ret) {
			case RelayBoardClient::INIT_CONFIG_CHANGED:
			case RelayBoardClient::INIT_CONFIG_FAILED:
				m_iComState = neo_msgs2::msg::RelayBoardV2::CS_CONFIGURATION_FAILED;
				break;
			default:
				m_iComState = neo_msgs2::msg::RelayBoardV2::CS_ERROR;
		}

		if (ret == RelayBoardClient::INIT_OPEN_PORT_FAILED)
		{
			RCLCPP_ERROR(this->get_logger(),"INIT_OPEN_PORT_FAILED");
		}
		else if (ret == RelayBoardClient::INIT_WRITE_FAILED)
		{
			RCLCPP_ERROR(this->get_logger(),"INIT_WRITE_FAILED");
		}
		else if (ret == RelayBoardClient::INIT_CONFIG_CHANGED)
		{
			RCLCPP_ERROR(this->get_logger(),"INIT_CONFIG_CHANGED");
		}
		else if (ret == RelayBoardClient::INIT_CONFIG_FAILED)
		{
			RCLCPP_ERROR(this->get_logger(),"INIT_CONFIG_FAILED");
		}
		else if (ret == RelayBoardClient::INIT_UNKNOWN_ERROR)
		{
			RCLCPP_ERROR(this->get_logger(),"INIT_UNKNOWN_ERROR");
		}
	}

	// Show Config
	for (int iMotorNr = 0; iMotorNr < 8; iMotorNr++)
	{
		m_Drives[iMotorNr].bmotor_avaliable = m_SerRelayBoard->getMotorAvailable(iMotorNr);
		m_Drives[iMotorNr].bmotor_homed = m_SerRelayBoard->getMotorHomed(iMotorNr);
		RCLCPP_INFO(this->get_logger(),"Drive %d: Active %s     Available %s", iMotorNr + 2, m_Drives[iMotorNr].bmotor_active ? "true " : "false", m_Drives[iMotorNr].bmotor_avaliable ? "true" : "false");
		RCLCPP_INFO(this->get_logger(),"         Homing %s     Homed %s", m_Drives[iMotorNr].bhoming_active ? "true " : "false", m_Drives[iMotorNr].bmotor_homed ? "true" : "false");
	}
	RCLCPP_INFO(this->get_logger(),"IOBoard: Active %s     Available %s", m_bIOBoardActive ? "true" : "false", m_SerRelayBoard->getIOBoardAvailable() ? "true" : "false");
	RCLCPP_INFO(this->get_logger(),"USBoard: Active %s     Available %s", m_bUSBoardActive ? "true" : "false", m_SerRelayBoard->getUSBoardAvailable() ? "true" : "false");

	//----------------------------------------Init Publisher/Subscriber--------------------------------------------

	// topics and subscriber which will allways get published
	topicPub_isEmergencyStop = this->create_publisher<neo_msgs2::msg::EmergencyStopState>("emergency_stop_state", 1);
	topicPub_RelayBoardState = this->create_publisher<neo_msgs2::msg::RelayBoardV2>("state", 1);
	topicPub_BatteryState = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 1);

	// ---- Service Callbacks ---
	this->srv_SetEMStop = this->create_service<neo_srvs2::srv::RelayBoardSetEMStop>("set_EMstop", std::bind(&NeoRelayBoardNode::serviceRelayBoardSetEmStop, this, _1, _2));
	this->srv_UnSetEMStop = this->create_service<neo_srvs2::srv::RelayBoardUnSetEMStop>("unset_EMstop", std::bind(&NeoRelayBoardNode::serviceRelayBoardUnSetEmStop, this, _1, _2));
	this->srv_SetRelay = this->create_service<neo_srvs2::srv::RelayBoardSetRelay>("set_relay", std::bind(&NeoRelayBoardNode::serviceRelayBoardSetRelay, this, _1, _2));
	this->srv_StartCharging = this->create_service<std_srvs::srv::Empty>("start_charging", std::bind(&NeoRelayBoardNode::serviceStartCharging, this,_1,_2));
	this->srv_StopCharging = this->create_service<std_srvs::srv::Empty>("stop_charging", std::bind(&NeoRelayBoardNode::serviceStopCharging, this,_1,_2));
	this->srv_SetLCDMsg = this->create_service<neo_srvs2::srv::RelayBoardSetLCDMsg>("set_LCD_msg", std::bind(&NeoRelayBoardNode::serviceRelayBoardSetLCDMsg, this, _1, _2));

	if (m_iactive_motors != 0)
	{
		topicPub_drives = this->create_publisher<sensor_msgs::msg::JointState>("drives/joint_states", 1);
		topicSub_drives = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("drives/joint_trajectory", 1, std::bind(&NeoRelayBoardNode::getNewVelocitiesFomTopic, this,_1));
	}

	if (m_bIOBoardActive)
	{
		topicPub_IOBoard = this->create_publisher<neo_msgs2::msg::IOBoard>("ioboard/data", 1);
		this->srv_SetDigOut = this->create_service<neo_srvs2::srv::IOBoardSetDigOut>("ioboard/set_digital_output", std::bind(&NeoRelayBoardNode::serviceIOBoardSetDigOut, this, _1, _2));
	}

	if (m_bUSBoardActive)
	{
		topicPub_usBoard = this->create_publisher<neo_msgs2::msg::USBoard>("usboard/measurements", 1);

		for (int i = 0; i < 16; ++i)
		{
			if(m_bUSBoardSensorActive[i]) {
				topicPub_USRangeSensor[i] = this->create_publisher<sensor_msgs::msg::Range>("usboard/sensor" + std::to_string(i + 1), 1);
			}
		}
	}
	
	return 0;
}

//--------------------------RelayBoardV2-----------------------------------------------------------------------

int NeoRelayBoardNode::HandleCommunication()
{
	// if relayboard is not available return
	if (!m_bRelayBoardV2Available)
		return m_iComState;

	auto now = rclcpp::Clock().now();

	// check for input timeout
	if ((now - m_last_trajectory_time).seconds() > m_trajectory_timeout)
	{
		if (!is_trajectory_timeout && !m_last_trajectory_time.seconds() == 0) {
			RCLCPP_WARN_STREAM(this->get_logger(),"joint_trajectory input timeout! Stopping now.");
		}
		is_trajectory_timeout = true;
	}
	else {
		is_trajectory_timeout = false;
	}

	if (is_trajectory_timeout) {
		for (int i = 0; i < m_imotor_count; i++) {
			m_SerRelayBoard->setMotorDesiredEncS(i, 0);		// set velocities to zero
		}
	}

	// send current data to relayboard
	const int iTXReturn = m_SerRelayBoard->sendDataToRelayBoard();

	// check if sending was ok
	if (iTXReturn == RelayBoardClient::TX_OK)
	{
		// try to receive current data from relayboard
		const int iRXReturn = m_SerRelayBoard->evalRxBuffer();

		// record current time stamp for latest message data
		m_tCurrentTimeStamp = rclcpp::Clock().now();

		// check if data was received
		if (iRXReturn == m_iLastRXReturn)
		{
			// Do not show message again
		}

		else if (iRXReturn == RelayBoardClient::RX_UPDATE_MSG) // ok
		{
			RCLCPP_INFO(this->get_logger(),"Communicating with RelayBoard");
		}
		else if (iRXReturn == RelayBoardClient::RX_TIMEOUT) // No Answer => resend
		{
			RCLCPP_ERROR(this->get_logger(),"No answer from RelayBoard (Timeout) ... ");
		}
		else if (iRXReturn == RelayBoardClient::RX_WRONG_CHECKSUM)
		{
			RCLCPP_ERROR(this->get_logger(),"Wrong checksum");
		}
		else if (iRXReturn == RelayBoardClient::RX_NO_HEADER)
		{
			RCLCPP_ERROR(this->get_logger(),"No valid message header found");
		}
		else if (iRXReturn == RelayBoardClient::RX_ERROR_MSG)
		{
			RCLCPP_ERROR(this->get_logger(),"RelayBoard error");
		}
		else
		{
			// Unknown error
			RCLCPP_ERROR_STREAM(this->get_logger(),"Unknown error: " << iRXReturn);
		}
		m_iLastRXReturn = iRXReturn;
	}
	else
	{
		// sending failed
		m_iComState = neo_msgs2::msg::RelayBoardV2::CS_ERROR;
		RCLCPP_ERROR(this->get_logger(),"Failed to send data");
	}

	return m_iComState;
}

// -------Publisher------

void NeoRelayBoardNode::PublishRelayBoardState()
{
	if (!m_bRelayBoardV2Available)
	{
		neo_msgs2::msg::RelayBoardV2 relayboardv2_msg;
		relayboardv2_msg.header.stamp = this->now();

		// important part for COM_CONFIG_FAILED
		relayboardv2_msg.communication_state = m_iComState;

		// publish neo_msgs2::msg::RelayBoardV2
		topicPub_RelayBoardState->publish(relayboardv2_msg);
	}
	else
	{
		// RelayBoardV2 is available
		neo_msgs2::msg::RelayBoardV2 relayboardv2_msg;
		relayboardv2_msg.header.stamp = m_tCurrentTimeStamp;

		int iState = 0;
		m_SerRelayBoard->getRelayBoardState(&iState);

		relayboardv2_msg.relayboardv2_state.fill(false);		  // initialize all states with false
		relayboardv2_msg.relayboardv2_state[0] = (iState == 0);	  // no error
		relayboardv2_msg.relayboardv2_state[1] = (iState & 512);  // charging relay error
		relayboardv2_msg.relayboardv2_state[2] = (iState & 64);   // release brakes button failed
		relayboardv2_msg.relayboardv2_state[3] = (iState & 4);	  // motor error
		relayboardv2_msg.relayboardv2_state[4] = (iState & 8);    // safety relay error
		relayboardv2_msg.relayboardv2_state[5] = (iState & 16);   // Leistungsrelais error
		relayboardv2_msg.relayboardv2_state[6] = (iState & 32);   // EMStop system error

		relayboardv2_msg.shutdown = (iState & 0x400); // relayboard is powering of in < 30s

		relayboardv2_msg.communication_state = m_iComState;

		int iChargingState = 0;
		m_SerRelayBoard->getChargingState(&iChargingState);
		relayboardv2_msg.charging_state = iChargingState;

		int iTemperature = 0;
		m_SerRelayBoard->getTemperature(&iTemperature);
		relayboardv2_msg.temperature = iTemperature;

		int iBatteryVoltage = 0;
		m_SerRelayBoard->getBattVoltage(&iBatteryVoltage);
		relayboardv2_msg.battery_voltage = iBatteryVoltage / 1000.f; // [mV] => [v]

		int iChargingCurrent = 0;
		m_SerRelayBoard->getChargingCurrent(&iChargingCurrent);
		relayboardv2_msg.charging_current = iChargingCurrent / 10.f; // [A]

		relayboardv2_msg.relay_states[0] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_1);
		relayboardv2_msg.relay_states[1] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_2);
		relayboardv2_msg.relay_states[2] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_3);
		relayboardv2_msg.relay_states[3] = m_SerRelayBoard->getRelayBoardDigOutState(m_SerRelayBoard->RELAY_ON_DEMAND_4);

		relayboardv2_msg.keypad[0] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_INFO);
		relayboardv2_msg.keypad[1] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_HOME);
		relayboardv2_msg.keypad[2] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_START);
		relayboardv2_msg.keypad[3] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_STOP);
		relayboardv2_msg.keypad[4] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_RELEASE_BRAKE);
		relayboardv2_msg.keypad[5] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_1);
		relayboardv2_msg.keypad[6] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_2);
		relayboardv2_msg.keypad[7] = m_SerRelayBoard->getKeyState(m_SerRelayBoard->KEY_ON_DEMAND_3);

		// publish neo_msgs2::msg::RelayBoardV2
		topicPub_RelayBoardState->publish(relayboardv2_msg);

		// handle RelayBoardV2 shutdown
		// RelayBoardV2 will power off in < 30 sec
		if (iState & 0x400)
		{
			RCLCPP_INFO(this->get_logger(),"-----------SHUTDOWN Signal from RelayBoardV2----------");
			rclcpp::shutdown();
			usleep(2000);
			system("sudo halt -p");
		}
	}
}

void NeoRelayBoardNode::PublishBatteryState()
{
	if (!m_bRelayBoardV2Available)
		return;

	sensor_msgs::msg::BatteryState bstate_msg;
	bstate_msg.header.stamp = m_tCurrentTimeStamp;

	// get battery voltage from relayboardv2 msg
	int iBatteryVoltage = 0;
	float fBatteryVoltage = 0.0;
	float fBatteryPercentage = 0;
	m_SerRelayBoard->getBattVoltage(&iBatteryVoltage);
	fBatteryVoltage = iBatteryVoltage / 1000.f;

	if(fBatteryVoltage > 36) {
		const float vmin = 44.0;
		const float vmax = 49.5;
		fBatteryPercentage = fmin(fmax((fBatteryVoltage - vmin) / (vmax - vmin), 0), 1);
	} else if(fBatteryVoltage > 18) {
		const float vmin = 22.5;
		const float vmax = 25.0;
		fBatteryPercentage = fmin(fmax((fBatteryVoltage - vmin) / (vmax - vmin), 0), 1);
	} else if(fBatteryVoltage >= 0 && fBatteryVoltage <= 1.001) {
		fBatteryPercentage = fBatteryVoltage;
		fBatteryVoltage = 48;
	}

	// get charging state from relayboardv2 msg
	int iChargingState = 0;
	m_SerRelayBoard->getChargingState(&iChargingState);

	/* power_supply_status: (uint8   The charging status as reported.)
     * POWER_SUPPLY_STATUS_UNKNOWN
     * POWER_SUPPLY_STATUS_CHARGING
     * POWER_SUPPLY_STATUS_DISCHARGING
     * POWER_SUPPLY_STATUS_NOT_CHARGING
     * POWER_SUPPLY_STATUS_FULL
    */

	// power_supply_status only supports very few different states
	if (iChargingState == m_SerRelayBoard->CHS_CHARGING)
	{
		bstate_msg.power_supply_status = bstate_msg.POWER_SUPPLY_STATUS_CHARGING;
	}
	else
	{
		bstate_msg.power_supply_status = bstate_msg.POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	// get charging current from relayboardv2 msg
	int iCurrent = 0;
	m_SerRelayBoard->getChargingCurrent(&iCurrent);

	bstate_msg.header.stamp = m_tCurrentTimeStamp;
	bstate_msg.header.frame_id = "";

	bstate_msg.voltage = fBatteryVoltage;							 // float32 Voltage in Volts (Mandatory)
	bstate_msg.current = iCurrent / 10.f;									 // float32 Negative when discharging (A)  (If unmeasured NaN)
	bstate_msg.charge = NAN;												 // float32 Current charge in Ah  (If unmeasured NaN)
	bstate_msg.capacity = NAN;												 // float32 Capacity in Ah (last full capacity)  (If unmeasured NaN)
	bstate_msg.design_capacity = m_fBatteryDesignCapacity;					 // float32 Capacity in Ah (design capacity)  (If unmeasured NaN)
	bstate_msg.percentage = fBatteryPercentage;								 // float32 Charge percentage on 0 to 1 range  (If unmeasured NaN)
	bstate_msg.power_supply_health = bstate_msg.POWER_SUPPLY_HEALTH_UNKNOWN; // uint8   The battery health metric.
	bstate_msg.power_supply_technology = m_iBatteryChemistry;				 // uint8   The battery chemistry.
	bstate_msg.present = true;												 // bool    True if the battery is present
	//bstate.cell_voltage[]                                 // float32[] An array of individual cell voltages for each cell in the pack
	// If individual voltages unknown but number of cells known set each to NaN
	bstate_msg.location = m_sBatteryLocation.c_str();		   // The location into which the battery is inserted. (slot number or plug)
	bstate_msg.serial_number = m_sBatterySerialNumber.c_str(); // The best approximation of the battery serial number

	topicPub_BatteryState->publish(bstate_msg);
}

void NeoRelayBoardNode::PublishEmergencyStopStates()
{
	if (!m_bRelayBoardV2Available)
		return;

	bool EM_signal;
	double duration_since_EM_confirmed;

	neo_msgs2::msg::EmergencyStopState EM_msg;
	EM_msg.header.stamp = m_tCurrentTimeStamp;

	// assign input (laser, button) specific EM state
	EM_msg.emergency_button_stop = m_SerRelayBoard->isEMStop();
	EM_msg.scanner_stop = m_SerRelayBoard->isScannerStop();

	// determine current EMStopState
	EM_signal = (EM_msg.emergency_button_stop || EM_msg.scanner_stop);

	switch (m_iEM_stop_state)
	{
	case ST_EM_FREE:
	{
		if (EM_signal == true)
		{
			RCLCPP_ERROR(this->get_logger(),"Emergency stop was issued");
			m_iEM_stop_state = EM_msg.EMSTOP;
		}
		break;
	}
	case ST_EM_ACTIVE:
	{
		if (EM_signal == false)
		{
			RCLCPP_INFO(this->get_logger(),"Emergency stop was confirmed");
			m_iEM_stop_state = EM_msg.EMCONFIRMED;
			m_time_of_EM_confirmed = rclcpp::Clock().now();
		}
		break;
	}
	case ST_EM_CONFIRMED:
	{
		if (EM_signal == true)
		{
			RCLCPP_ERROR(this->get_logger(),"Emergency stop was issued");
			m_iEM_stop_state = EM_msg.EMSTOP;
		}
		else
		{
			duration_since_EM_confirmed = (rclcpp::Clock().now() - m_time_of_EM_confirmed).seconds();
			if (duration_since_EM_confirmed > m_duration_for_EM_free)
			{
				RCLCPP_INFO(this->get_logger(),"Emergency stop released");
				m_iEM_stop_state = EM_msg.EMFREE;
			}
		}
		break;
	}
	};

	EM_msg.emergency_state = m_iEM_stop_state;
	EM_msg.software_stop = m_bSoftware_EM_stop;

	topicPub_isEmergencyStop->publish(EM_msg);
}

//#############################################
//       RelayBoardV2 Service Callbacks
//#############################################
bool NeoRelayBoardNode::serviceRelayBoardSetEmStop(const std::shared_ptr<neo_srvs2::srv::RelayBoardSetEMStop::Request> /*req*/, std::shared_ptr<neo_srvs2::srv::RelayBoardSetEMStop::Response> res)
{
	if (m_SerRelayBoard->isSoftEMStop())
	{
		RCLCPP_INFO(this->get_logger(),"Already in SoftEMStop");
	}
	else
	{
		m_SerRelayBoard->setSoftEMStop();
		RCLCPP_INFO(this->get_logger(),"Enabled SoftEMStop");
		res->success = true;
		m_bSoftware_EM_stop = true;
	}
	return true;
}

bool NeoRelayBoardNode::serviceRelayBoardUnSetEmStop(const std::shared_ptr<neo_srvs2::srv::RelayBoardUnSetEMStop::Request> /*req*/, std::shared_ptr<neo_srvs2::srv::RelayBoardUnSetEMStop::Response> res)
{
	if (!m_SerRelayBoard->isSoftEMStop())
	{
		RCLCPP_INFO(this->get_logger(),"Already not in SoftEMStop");
	}
	else
	{
		m_SerRelayBoard->unsetSoftEMStop();
		RCLCPP_INFO(this->get_logger(),"Released SoftEMStop");
		res->success = true;
		m_bSoftware_EM_stop = false;
	}
	return true;
}

bool NeoRelayBoardNode::serviceRelayBoardSetRelay(std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> req,
												  std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Response> res)
{
	if (!m_bRelayBoardV2Available)
	{
		res->success = false;
		return false;
	}
	else
	{
		// check if Relay ID is valid
		if (req->id >= 0 && req->id < 4)
		{
			m_SerRelayBoard->setRelayBoardDigOut(req->id, req->state);
			res->success = true;
			return true;
		}
	}
	res->success = false;
	return false;
}

bool NeoRelayBoardNode::serviceStartCharging(std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/, std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
	if (m_bRelayBoardV2Available)
	{
		m_SerRelayBoard->startCharging();
		return true;
	}
	return false;
}

bool NeoRelayBoardNode::serviceStopCharging(std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/, std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
	if (m_bRelayBoardV2Available)
	{
		m_SerRelayBoard->stopCharging();
		return true;
	}
	return false;
}

bool NeoRelayBoardNode::serviceRelayBoardSetLCDMsg(std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Request> req,
												   std::shared_ptr<neo_srvs2::srv::RelayBoardSetLCDMsg::Response> res)
{
	if (m_bRelayBoardV2Available)
	{
		m_SerRelayBoard->writeLCD(req->message.c_str());
		res->success = true;
		return true;
	}
	res->success = false;
	return false;
}

//#############################################
//       RelayBoardV2 Motor Control
//#############################################

void NeoRelayBoardNode::PublishJointStates()
{
	if (!m_bRelayBoardV2Available)
		return;

	if(!m_JointStates)
		return;

	long lEnc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	long lEncS[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int iStatus[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	static float sfLastPos[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	sensor_msgs::msg::JointState state;
	state.header.stamp = m_tCurrentTimeStamp - rclcpp::Duration::from_seconds(m_tMotorDelay);

	// Publish Data for all possible Motors
	state.name.resize(m_drivesNr);
	state.position.resize(m_drivesNr);
	state.velocity.resize(m_drivesNr);

	// TODO Joint Names einfügen
	// for(int i = 0; i<anz_drives; i++)  state.name[i] = joint_names[i];

	// Motor Data from MSG Handler for each Motor
	// Enc (4 Byte), EncS (4 Byte) and Status (2 Byte) for each Motor
	for (int i = 0; i < m_drivesNr; i++)
	{
		state.name[i] = m_Drives[i].sName.c_str();
		m_SerRelayBoard->getMotorEnc(i, &lEnc[i]);
		m_SerRelayBoard->getMotorEncS(i, &lEncS[i]);
		m_SerRelayBoard->getMotorState(i, &iStatus[i]);
		state.position[i] = lEnc[i] - sfLastPos[i];
		sfLastPos[i] = lEnc[i];
		//RCLCPP_INFO(this->get_logger(),"Motor %d: Enc: %f",i,(float)lEnc[i]);
		state.velocity[i] = m_Drives[i].iSign * m_Drives[i].convIncrPerPeriodToRadS(lEncS[i]);
	}

	topicPub_drives->publish(state);
}

void NeoRelayBoardNode::getNewVelocitiesFomTopic(const trajectory_msgs::msg::JointTrajectory::SharedPtr jt)
{
	if (!m_bRelayBoardV2Available)
		return;

	trajectory_msgs::msg::JointTrajectoryPoint point = jt->points[0];

	for (int i = 0; i < m_imotor_count; i++)
	{
		// convert velocities [rad/s] -> [incr/period]
		//RCLCPP_INFO(this->get_logger(),"Motor: %d ; Vel: %d [rad]; Vel: %d [incr/period]",i,point.velocities[i],dvelocity);
		double dvelocity = m_Drives[i].iSign * m_Drives[i].convRadSToIncrPerPeriod(point.velocities[i]);

		// check if velocity is too high -> limit velocity
		/*if(MathSup::limit((int)&dvelocity, (int)Drives[i].getVelMax()) != 0)
        {
            RCLCPP_ERROR(this->get_logger(),"Velocity for motor %d limited",i+2);
        }*/

		// send Data to MSG Handler
		m_SerRelayBoard->setMotorDesiredEncS(i, dvelocity);
	}

	m_last_trajectory_time = rclcpp::Clock().now();
}

//-----------------------------USBoard-------------------------------------------------------------------------

void NeoRelayBoardNode::PublishUSBoardData()
{
	if (!m_bRelayBoardV2Available || !m_bUSBoardActive)
		return;

	int usSensors1[8];
	int usSensors2[8];
	int usAnalog[4];

	m_SerRelayBoard->getUSBoardData1To8(usSensors1);
	m_SerRelayBoard->getUSBoardData9To16(usSensors2);
	m_SerRelayBoard->getUSBoardAnalogIn(usAnalog);

	neo_msgs2::msg::USBoard usBoard;
	usBoard.header.stamp = m_tCurrentTimeStamp;

	for (int i = 0; i < 8; i++)
	{
		usBoard.sensor[i] = usSensors1[i];
	}
	for (int i = 0; i < 8; i++)
	{
		usBoard.sensor[i + 8] = usSensors2[i];
	}
	for (int i = 0; i < 4; i++)
	{
		usBoard.analog[i] = usAnalog[i];
	}

	// Publish raw data in neo_msgs2::msg::USBoard format
	topicPub_usBoard->publish(usBoard);

	// Additionally publish data in ROS sensor_msgs::Range format
	for (int i = 0; i < 16; ++i)
	{
		if(!m_bUSBoardSensorActive[i]) {
			continue;
		}
		std_msgs::msg::Header header;
		header.stamp = m_tCurrentTimeStamp;						   // time
		header.frame_id = "us_" + std::to_string(i + 1) + "_link"; 	// string

		sensor_msgs::msg::Range us_range_msg;
		us_range_msg.header = header;
		us_range_msg.radiation_type = 0;				// uint8   => Enum ULTRASOUND=0; INFRARED=1
		us_range_msg.field_of_view = 1.05;				// float32 [rad]
		us_range_msg.min_range = 0.1;					// float32 [m]
		us_range_msg.max_range = 1.2;					// float32 [m]
		us_range_msg.range = usBoard.sensor[i] / 100.f; // float32 [cm] => [m]

		topicPub_USRangeSensor[i]->publish(us_range_msg);
	}
}

/*void NeoRelayBoardNode::startUSBoard(const std_msgs::Int16& configuration)
{
    if(!m_bRelayBoardV2Available || !m_ihasUSBoard) return;
	m_SerRelayBoard->startUSBoard(configuratiothis->data);
}

void NeoRelayBoardNode::stopUSBoard(const std_msgs::Empty& empty)
{
    if(!m_bRelayBoardV2Available || !m_ihasUSBoard) return;
	m_SerRelayBoard->stopUSBoard();
}*/

//---------------------------------IOBoard---------------------------------------------------------------------

void NeoRelayBoardNode::PublishIOBoard()
{
	if (!m_bRelayBoardV2Available || !m_bIOBoardActive)
		return;

	neo_msgs2::msg::IOBoard msg_IOBoard;
	msg_IOBoard.header.stamp = m_tCurrentTimeStamp;

	//bool[16] digital_inputs			# state for all digital inputs
	//bool[16] digital_outputs          # state for all digital outputs
	//uint8[4] analog_inputs			# analog input values

	int iDigInData = 0;
	m_SerRelayBoard->getIOBoardDigIn(&iDigInData);

	int iDigOutData = 0;
	m_SerRelayBoard->getIOBoardDigOut(&iDigOutData);

	for (int iIOCnt = 0; iIOCnt < 16; iIOCnt++)
	{
		const int iMask = 1 << iIOCnt;
		msg_IOBoard.digital_inputs[iIOCnt] = bool(iDigInData & iMask);
		msg_IOBoard.digital_outputs[iIOCnt] = bool(iDigOutData & iMask);
	}

	int analog_in[8];
	m_SerRelayBoard->getIOBoardAnalogIn(analog_in);
	for (int i = 0; i < 8; i++)
	{
		msg_IOBoard.analog_inputs[i] = analog_in[i];
	}

	topicPub_IOBoard->publish(msg_IOBoard);
}

bool NeoRelayBoardNode::serviceIOBoardSetDigOut(std::shared_ptr<neo_srvs2::srv::IOBoardSetDigOut::Request> req,
												std::shared_ptr<neo_srvs2::srv::IOBoardSetDigOut::Response> res)
{
	if (!m_bRelayBoardV2Available || !m_bIOBoardActive)
	{
		res->success = false;
		return false;
	}
	else
	{
		// check if DigOut ID is valid
		if (req->id >= 0 && req->id < 16)
		{
			m_SerRelayBoard->setIOBoardDigOut(req->id, req->state);
			res->success = true;
			return true;
		}
	}
	res->success = false;
	return false;
}

//-----------------------------END IOBoard---------------------------------------------------------------------
