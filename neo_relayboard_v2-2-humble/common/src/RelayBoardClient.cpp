/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Neobotix GmbH
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <linux/serial.h>

#include "rclcpp/rclcpp.hpp"

#include "../include/RelayBoardClient.h"

#define RS422_BAUDRATE 420000


RelayBoardClient::RelayBoardClient()
{
	fd = -1;

	m_iNumBytesRec = 0;

	m_bRelayData = true;		// always send relay data
	m_bLCDData = false;
	m_bIOBoardData = false;
	m_bUSBoardData = false;
	m_bSpeakerData = true;		// always send speaker data
	m_bChargeData = false;

	m_REC_MSG.iRelayBoard_Status = 0;
	m_REC_MSG.iCharging_Current = 0;
	m_REC_MSG.iCharging_State = 0;
	m_REC_MSG.iBattery_Voltage = 0;
	m_REC_MSG.iKey_Pad = 0;
	m_REC_MSG.iTemp_Sensor = 0;
	m_REC_MSG.iIODig_In = 0;
	m_REC_MSG.iIOBoard_State = 0;
	m_REC_MSG.iUSBoard_State = 0;

	for (int a = 0; a < 8; a++)
	{
		m_Motor[a].bActive = false;
		m_Motor[a].bAvailable = false;
		m_Motor[a].bHoming = false;
		m_Motor[a].bHomed = false;
		m_Motor[a].lEnc = 0.0;
		m_Motor[a].lEncS = 0.0;
		m_Motor[a].iStatus = 0;
		m_Motor[a].ldesiredEncS = 0.0;
	}

	m_IOBoard.bActive = false;
	m_IOBoard.bAvailable = false;

	m_USBoard.bActive = false;
	m_USBoard.bAvailable = false;

	m_S_MSG.iSoftEM = 0;
	m_S_MSG.iCmdRelayBoard = 0;
	m_S_MSG.IOBoard_Cmd = 0;
	m_S_MSG.USBoard_Cmd = 0;
	m_S_MSG.Speaker = 0;
	m_S_MSG.SpeakerLoud = 0;

	m_blogging = false;
}

RelayBoardClient::~RelayBoardClient()
{
	if (fd >= 0)
	{
		close(fd);
	}
}

bool RelayBoardClient::waitForRx(uint64_t timeout_usec)
{
	fd_set read_set;
	FD_ZERO(&read_set);
	FD_SET(fd, &read_set);

	struct timeval timeout = {int32_t(timeout_usec / 1000000), int32_t(timeout_usec % 1000000)};
	return select(fd + 1, &read_set, 0, 0, &timeout) == 1;		// wait for input, with a timeout
}

int RelayBoardClient::evalRxBuffer(uint64_t timeout_usec)
{
	int msg_type = -1;

	unsigned char cDat[4096] = {};
	unsigned char cHeader[4] = {};

	// read header
	while (true)
	{
		cHeader[3] = cHeader[2];
		cHeader[2] = cHeader[1];
		cHeader[1] = cHeader[0];

		if(!waitForRx(timeout_usec))
		{
			return RX_TIMEOUT;
		}

		const int num_bytes_read = read(fd, cHeader, 1);

		if (num_bytes_read < 1)
		{
			return RX_NO_HEADER;	// error or shutdown
		}

		if ((cHeader[3] == 0x08) && (cHeader[2] == 0xFE) && (cHeader[1] == 0xEF) && (cHeader[0] == 0x08))
		{
			// Update Msg
			msg_type = 1;
			break;
		}
		else if ((cHeader[3] == 0x02) && (cHeader[2] == 0x80) && (cHeader[1] == 0xD6) && (cHeader[0] == 0x02))
		{
			// Config Msg
			msg_type = 2;
			break;
		}
		else if ((cHeader[3] == 0x02) && (cHeader[2] == 0xFF) && (cHeader[1] == 0xD6) && (cHeader[0] == 0x02))
		{
			// Error Msg
			msg_type = 3;
			break;
		}
	}

	// look up number of bytes to read
	int payload_size = 0;
	switch (msg_type)
	{
	case 1:
		payload_size = m_iNumBytesRec - 4;
		break;
	case 2:
		payload_size = 6;
		break;
	case 3:
		payload_size = 3;
		break;
	default:
		return RX_UNKNOWN_MSG;
	}

	// read message payload data
	{
		int offset = 0;
		int num_bytes_left = payload_size;

		while (num_bytes_left > 0)
		{
			if(!waitForRx(timeout_usec))
			{
				return RX_TIMEOUT;
			}

			const int num_bytes_read = read(fd, cDat + offset, num_bytes_left);

			if (num_bytes_read <= 0)
			{
				if(rclcpp::ok()) {
					// //ROS_ERROR_STREAM("read() failed with: " << std::strerror(errno));
				}
				return RX_UNKNOWN_ERROR;
			}

			offset += num_bytes_read;
			num_bytes_left -= num_bytes_read;

			// discard any leftover data, to make sure we don't accumulate
			while(waitForRx(0)) {
				char dummy[1] = {};
				if(read(fd, dummy, 1) != 1) {
					break;
				}
			}
		}
	}

	// //ROS_DEBUG("Received %d bytes payload for msg_type %d", payload_size, msg_type);

	// log data to file
	if (m_blogging == true)
	{
		log_to_file(2, cDat, payload_size); // direction 1 = transmitted; 2 = recived
	}

	// calc checksum
	int my_checksum = 0;
	int received_checksum = 0;

	my_checksum += cHeader[3];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[2];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[1];
	my_checksum %= 0xFF00;
	my_checksum += cHeader[0];
	for (int e = 0; e < payload_size - 2; e++)
	{
		my_checksum %= 0xFF00;
		my_checksum += cDat[e];
	}

	// read checksum from msg
	received_checksum = (cDat[payload_size - 1] << 8);
	received_checksum += cDat[payload_size - 2];

	// check if checksum is equal to received checksum
	if (received_checksum != my_checksum)
	{
		return RX_WRONG_CHECKSUM;
	}

	// handle msg (depends on msg type)
	if (msg_type == 1)
	{
		// Update Msg
		convRecMsgToData(cDat);
		return RX_UPDATE_MSG;
	}
	else if (msg_type == 2)
	{
		// Config Msg
		m_iFoundMotors = cDat[0];
		m_iHomedMotors = cDat[1];
		m_iFoundExtHardware = cDat[2];
		m_iConfigured = cDat[3];

		// Set motors available
		m_Motor[7].bAvailable = m_iFoundMotors & 0x80;
		m_Motor[6].bAvailable = m_iFoundMotors & 0x40;
		m_Motor[5].bAvailable = m_iFoundMotors & 0x20;
		m_Motor[4].bAvailable = m_iFoundMotors & 0x10;
		m_Motor[3].bAvailable = m_iFoundMotors & 0x8;
		m_Motor[2].bAvailable = m_iFoundMotors & 0x4;
		m_Motor[1].bAvailable = m_iFoundMotors & 0x2;
		m_Motor[0].bAvailable = m_iFoundMotors & 0x1;

		// Set motors homed
		m_Motor[7].bHomed = m_iHomedMotors & 0x80;
		m_Motor[6].bHomed = m_iHomedMotors & 0x40;
		m_Motor[5].bHomed = m_iHomedMotors & 0x20;
		m_Motor[4].bHomed = m_iHomedMotors & 0x10;
		m_Motor[3].bHomed = m_iHomedMotors & 0x8;
		m_Motor[2].bHomed = m_iHomedMotors & 0x4;
		m_Motor[1].bHomed = m_iHomedMotors & 0x2;
		m_Motor[0].bHomed = m_iHomedMotors & 0x1;

		// Set ext hardware available
		m_USBoard.bAvailable = m_iFoundExtHardware & 0x2;
		m_IOBoard.bAvailable = m_iFoundExtHardware & 0x1;

		// //ROS_INFO("Found Motors: %#04x", cDat[0]);
		// //ROS_INFO("Homed Motors: %#04x", cDat[1]);
		// //ROS_INFO("Ext Hardware: %#04x", cDat[2]);
		// //ROS_INFO("Configured:   %#04x", cDat[3]);

		return RX_CONFIG_MSG;
	}
	else if (msg_type == 3)
	{
		// Error Msg
		// //ROS_ERROR_STREAM("RX_ERROR_MSG: error code = '" << char(cDat[0]) << "'");
		return RX_ERROR_MSG;
	}

	return RX_UNKNOWN_ERROR;
}

// ----------------------------------Configuration--------------------------------------

int RelayBoardClient::init(const char *cdevice_name, int iactive_motors, int ihoming_motors, int iext_hardware,
						   long lModulo0, long lModulo1, long lModulo2, long lModulo3,
						   long lModulo4, long lModulo5, long lModulo6, long lModulo7)
{
	if (fd >= 0)
	{
		close(fd);
	}

	// open serial port
	fd = open(cdevice_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0)
	{
		//ROS_ERROR("Failed to open serial port: errno %d", errno);
		return INIT_OPEN_PORT_FAILED;
	}

	// configure serial port
	{
		termios options = {};
		tcgetattr(fd, &options); // get current config

		cfmakeraw(&options); // configure for binary data

		options.c_cflag &= ~CSTOPB;		   // one stop bit
		options.c_cflag &= ~CRTSCTS;	   // no hardware handshake
		options.c_cflag |= CLOCAL | CREAD; // legacy magic

		options.c_cc[VMIN] = 1;  // read at least one byte per read() call
		options.c_cc[VTIME] = 0; // inter-byte timeout (not needed in this case)

		cfsetispeed(&options, B38400); // this is needed for some reason
		cfsetospeed(&options, B38400); // this is needed for some reason

		tcsetattr(fd, TCSANOW, &options); // apply the new config
	}

	// set baudrate
	{
		struct serial_struct ss;
		ioctl(fd, TIOCGSERIAL, &ss);
		ss.flags |= ASYNC_SPD_CUST;
		ss.custom_divisor = ss.baud_base / RS422_BAUDRATE;
		ioctl(fd, TIOCSSERIAL, &ss);
	}

	unsigned char cConfig_Data[33]; // 4 Byte Header 3 Config Bytes 24 Byte Modulo 2 Byte Checksum
	int iChkSum = 0;

	// Header
	cConfig_Data[0] = 0x02;
	cConfig_Data[1] = 0x80;
	cConfig_Data[2] = 0xD6;
	cConfig_Data[3] = 0x02;
	//-------------configuration Bytes-----------------------------------
	cConfig_Data[4] = iactive_motors;
	cConfig_Data[5] = ihoming_motors;
	cConfig_Data[6] = iext_hardware;

	//-------------set motors active-------------------------------------
	m_Motor[7].bActive = iactive_motors & 0x80;
	m_Motor[6].bActive = iactive_motors & 0x40;
	m_Motor[5].bActive = iactive_motors & 0x20;
	m_Motor[4].bActive = iactive_motors & 0x10;
	m_Motor[3].bActive = iactive_motors & 0x8;
	m_Motor[2].bActive = iactive_motors & 0x4;
	m_Motor[1].bActive = iactive_motors & 0x2;
	m_Motor[0].bActive = iactive_motors & 0x1;

	//-------------set homeing active-------------------------------------
	m_Motor[7].bHoming = ihoming_motors & 0x80;
	m_Motor[6].bHoming = ihoming_motors & 0x40;
	m_Motor[5].bHoming = ihoming_motors & 0x20;
	m_Motor[4].bHoming = ihoming_motors & 0x10;
	m_Motor[3].bHoming = ihoming_motors & 0x8;
	m_Motor[2].bHoming = ihoming_motors & 0x4;
	m_Motor[1].bHoming = ihoming_motors & 0x2;
	m_Motor[0].bHoming = ihoming_motors & 0x1;

	//---------------Modulo for all Motors-------------------------------
	// Motor1
	cConfig_Data[7] = lModulo0 >> 16;
	cConfig_Data[8] = lModulo0 >> 8;
	cConfig_Data[9] = lModulo0;
	// Motor2
	cConfig_Data[10] = lModulo1 >> 16;
	cConfig_Data[11] = lModulo1 >> 8;
	cConfig_Data[12] = lModulo1;
	// Motor3
	cConfig_Data[13] = lModulo2 >> 16;
	cConfig_Data[14] = lModulo2 >> 8;
	cConfig_Data[15] = lModulo2;
	// Motor4
	cConfig_Data[16] = lModulo3 >> 16;
	cConfig_Data[17] = lModulo3 >> 8;
	cConfig_Data[18] = lModulo3;
	// Motor5
	cConfig_Data[19] = lModulo4 >> 16;
	cConfig_Data[20] = lModulo4 >> 8;
	cConfig_Data[21] = lModulo4;
	// Motor6
	cConfig_Data[22] = lModulo5 >> 16;
	cConfig_Data[23] = lModulo5 >> 8;
	cConfig_Data[24] = lModulo5;
	// Motor7
	cConfig_Data[25] = lModulo6 >> 16;
	cConfig_Data[26] = lModulo6 >> 8;
	cConfig_Data[27] = lModulo6;
	// Motor8
	cConfig_Data[28] = lModulo7 >> 16;
	cConfig_Data[29] = lModulo7 >> 8;
	cConfig_Data[30] = lModulo7;

	//-----------------Calc m_iNumBytesRec-----------------------------
	m_iNumBytesRec = 4;   // 4Byte Header
	m_iNumBytesRec += 13; // 11Byte ActRelayboardConfig, State, Charging Current, Charging State, Batt Voltage, Keypad, Temp

	int iNumDrives = 0;
	for (int g = 0; g < 8; g++)
	{
		if (m_Motor[g].bActive)
		{
			iNumDrives++;
		}
	}

	//ROS_DEBUG("Number of Drives %i", iNumDrives);
	m_iNumBytesRec += (iNumDrives * 10); // 10 Byte pro active Motor

	if (iext_hardware & 0x01) // IOBoard
	{
		//ROS_DEBUG("Ext. Hardware: IOBoard");
		m_IOBoard.bActive = true;
		m_iNumBytesRec += 20;
	}
	if (iext_hardware & 0x02) // USBoard
	{
		//ROS_DEBUG("Ext. Hardware: USBoard");
		m_USBoard.bActive = true;
		m_iNumBytesRec += 26;
	}

	m_iNumBytesRec += 2; // 2Byte Checksum
	//ROS_DEBUG("iNumBytesRec: %d", m_iNumBytesRec);

	//----------------Calc Checksum-------------------------------------
	for (int i = 4; i <= 30; i++) // i=4 => Header not added to Checksum
	{
		iChkSum %= 0xFF00;
		iChkSum += cConfig_Data[i];
	}
	// ----------------END Calc Checksum---------------------------------

	//----------------Add Checksum to Data------------------------------
	cConfig_Data[31] = iChkSum >> 8;
	cConfig_Data[32] = iChkSum;
	//------------END Add Checksum to Data------------------------------

	if (m_blogging == true)
	{
		log_to_file(3, cConfig_Data, sizeof(cConfig_Data)); // direction 1 = transmitted; 2 = recived ; 3 = config
	}

	const int bytes_written = write(fd, cConfig_Data, sizeof(cConfig_Data));

	if (bytes_written != sizeof(cConfig_Data))
	{
		//ROS_ERROR("FAILED: Could not write data to serial port!");
		return INIT_WRITE_FAILED;
	}

	const int evalRXreturn = evalRxBuffer();

	if (evalRXreturn == RX_CONFIG_MSG)
	{
		// Check received Data
		if (m_iConfigured == 1)
		{
			if (!checkConfig())
			{
				return INIT_CONFIG_FAILED;
			}
			//ROS_DEBUG("Configured");
			return INIT_CONFIG_OK;
		}
		else
		{
			//ROS_ERROR("FAILED: Invalid configuration");
			return INIT_CONFIG_FAILED;
		}
	}

	return INIT_UNKNOWN_ERROR;
}

void RelayBoardClient::getConfig(int *ActiveMotors, int *HomedMotors, int *ExtHardware, int *Configured)
{
	*ActiveMotors = m_iFoundMotors;
	*HomedMotors = m_iHomedMotors;
	*ExtHardware = m_iFoundExtHardware;
	*Configured = m_iConfigured;
}

bool RelayBoardClient::checkConfig()
{
	// returns true if config is ok
	// false otherwise

	bool config_ok = true;

	// Motor1 to Motor8
	for (int i = 0; i < 8; ++i)
	{
		if (m_Motor[i].bActive)
		{
			if (!m_Motor[i].bAvailable)
			{
				config_ok = false;
				//ROS_ERROR("Motor%d is active but NOT available!", i + 1);
			}
		}
		else
		{
			if (m_Motor[i].bAvailable)
			{
				//ROS_WARN("Motor%d is NOT active but available!", i + 1);
			}
		}
	}

	// IOBoard
	if (m_IOBoard.bActive)
	{
		if (!m_IOBoard.bAvailable)
		{
			config_ok = false;
			//ROS_ERROR("IOBoard is active but NOT available!");
		}
	}
	else
	{
		if (m_IOBoard.bAvailable)
		{
			//ROS_WARN("IOBoard is NOT active but available!");
		}
	}

	// USBoard
	if (m_USBoard.bActive)
	{
		if (!m_USBoard.bAvailable)
		{
			config_ok = false;
			//ROS_ERROR("USBoard is active but NOT available!");
		}
	}
	else
	{
		if (m_USBoard.bAvailable)
		{
			//ROS_WARN("USBoard is NOT active but available!");
		}
	}

	return config_ok;
}

bool RelayBoardClient::shutdownPltf()
{
	if (fd >= 0)
	{
		close(fd);
		fd = -1;
	}
	return true;
}

bool RelayBoardClient::isEMStop()
{
	return m_REC_MSG.iRelayBoard_Status & 0x01;
}

bool RelayBoardClient::isSoftEMStop()
{
	return m_S_MSG.iSoftEM == 1;
}

bool RelayBoardClient::isScannerStop()
{
	return m_REC_MSG.iRelayBoard_Status & 0x02;
}

int RelayBoardClient::sendDataToRelayBoard()
{
	int errorFlag = TX_OK;
	int iNrBytesWritten;
	unsigned char cMsg[4096];

	const int iNumBytesSend = convDataToSendMsg(cMsg);

	iNrBytesWritten = write(fd, cMsg, iNumBytesSend);

	if (iNrBytesWritten != iNumBytesSend)
	{
		errorFlag = TX_WRITE_FAILED;
		//ROS_ERROR("Serial port write failed!");
	}

	// log
	if (m_blogging == true)
	{
		log_to_file(1, cMsg, iNumBytesSend); // direction 1 = transmitted; 2 = recived
	}

	return errorFlag;
}

//---------------------------------------------------------------------------------------------------------------------
// MotorCtrl
//---------------------------------------------------------------------------------------------------------------------

void RelayBoardClient::setMotorDesiredEncS(int iMotorNr, long lEncS)
{
	m_Motor[iMotorNr].ldesiredEncS = lEncS;
}

void RelayBoardClient::getMotorEnc(int iMotorNr, long *lMotorEnc)
{
	*lMotorEnc = m_Motor[iMotorNr].lEnc;
}

void RelayBoardClient::getMotorEncS(int iMotorNr, long *lMotorEncS)
{
	*lMotorEncS = m_Motor[iMotorNr].lEncS;
}

void RelayBoardClient::getMotorState(int iMotorNr, int *iMotorStatus)
{
	*iMotorStatus = m_Motor[iMotorNr].iStatus;
}

bool RelayBoardClient::getMotorAvailable(int iMotorNr)
{
	return m_Motor[iMotorNr].bAvailable;
}

bool RelayBoardClient::getMotorHomed(int iMotorNr)
{
	return m_Motor[iMotorNr].bHomed;
}

//---------------------------------------------------------------------------------------------------------------------
// RelayBoard
//---------------------------------------------------------------------------------------------------------------------

void RelayBoardClient::getRelayBoardState(int *state)
{
	*state = m_REC_MSG.iRelayBoard_Status;
}

void RelayBoardClient::getRelayBoardDigOut(int *digOut)
{
	*digOut = m_S_MSG.iCmdRelayBoard;
}

bool RelayBoardClient::getRelayBoardDigOutState(int ID)
{
	/* Relay states are not evaluated from relayboardv2
     * -> All states are the same as was has been set
     */

	int iRelayStates = 0;
	getRelayBoardDigOut(&iRelayStates);

	// Charge Relay
	if (ID == RELAY_ON_DEMAND_1)
	{
		return iRelayStates & 8;
	}
	// On Demand Relay 1
	else if (ID == RELAY_ON_DEMAND_2)
	{
		return iRelayStates & 16;
	}
	// On Demand Relay 2
	else if (ID == RELAY_ON_DEMAND_3)
	{
		return iRelayStates & 32;
	}
	// On Demand Relay 3
	else if (ID == RELAY_ON_DEMAND_4)
	{
		return iRelayStates & 64;
	}

	return false;
}

void RelayBoardClient::setRelayBoardDigOut(int iChannel, bool bOn)
{
	/*Bit 4: Relais 1 schalten
	Bit 5: Relais 2 schalten
	Bit 6: Relais 3 schalten
	Bit 7: Relais 4 schalten*/

	m_bRelayData = true;

	if (iChannel >= 0 && iChannel < 4)
	{
		if (bOn)
		{
			m_S_MSG.iCmdRelayBoard |= (8 << iChannel);
		}
		else
		{
			m_S_MSG.iCmdRelayBoard &= ~(8 << iChannel);
		}
	}
}

void RelayBoardClient::getTemperature(int *temp)
{
	*temp = m_REC_MSG.iTemp_Sensor;
}

void RelayBoardClient::getBattVoltage(int *battvolt)
{
	*battvolt = m_REC_MSG.iBattery_Voltage;
}

void RelayBoardClient::getChargingCurrent(int *current)
{
	*current = m_REC_MSG.iCharging_Current;
}

void RelayBoardClient::getChargingState(int *state)
{
	*state = m_REC_MSG.iCharging_State;
}

void RelayBoardClient::writeLCD(const std::string &sText)
{
	m_bLCDData = true;

	for (int i = 0; i < 20; i++)
	{
		if (i < sText.size())
		{
			m_S_MSG.LCD_Txt[i] = sText[i];
		}
		else
		{
			m_S_MSG.LCD_Txt[i] = ' ';
		}
	}
}

void RelayBoardClient::getKeyPad(int *keypad)
{
	*keypad = m_REC_MSG.iKey_Pad;
}

bool RelayBoardClient::getKeyState(int ID)
{
	/* KEY PAD BIT CODED:
     * LSB : Info Taste gedrueckt
     * Bit 2: Home Taste gedrueckt
     * Bit 3: Start Taste gedrueckt
     * Bit 4: Stop Taste gedrueckt
     * Bit 5: Bremsen loesen Taste gedrueckt
     * Bit 6: On Demand Key 1
     * Bit 7: On Demand Key 2
     * Bit 8: On Demand Key 3
     * Bit 9: Reserviert
     * Bit 10: Reserviert
     * Bit 11: Reserviert
     * Bit 12: Reserviert
     * Bit 13: Reserviert
     * Bit 14: Reserviert
     * Bit 15: Reserviert
     * MSB: Reserviert
     */

	int iButtons = 0;
	getKeyPad(&iButtons);

	switch (ID)
	{
	case KEY_INFO:
		return (iButtons & 1); // Info Button
	case KEY_HOME:
		return (iButtons & 2); // Home Button
	case KEY_START:
		return (iButtons & 4); // Start Button
	case KEY_STOP:
		return (iButtons & 8); // Stop Button
	case KEY_RELEASE_BRAKE:
		return (iButtons & 16); // release brake button
	case KEY_ON_DEMAND_1:
		return (iButtons & 32); // on demand digital input
	case KEY_ON_DEMAND_2:
		return (iButtons & 64); // on demand digital input
	case KEY_ON_DEMAND_3:
		return (iButtons & 128); // on demand digital input
	}
	return false;
}

void RelayBoardClient::startCharging()
{
	m_bRelayData = true;
	m_S_MSG.iCmdRelayBoard |= 1;
}

void RelayBoardClient::stopCharging()
{
	m_bRelayData = true;
	m_S_MSG.iCmdRelayBoard &= ~1;
}

//-----------------------------------------------
// IOBoard
//-----------------------------------------------

bool RelayBoardClient::getIOBoardAvailable()
{
	return m_IOBoard.bAvailable;
}

void RelayBoardClient::getIOBoardDigIn(int *digIn)
{
	*digIn = m_REC_MSG.iIODig_In;
}

void RelayBoardClient::getIOBoardDigOut(int *digOut)
{
	*digOut = m_S_MSG.IOBoard_Cmd;
}

void RelayBoardClient::setIOBoardDigOut(int iChannel, bool bVal)
{
	const int iMask = (1 << iChannel);

	if (bVal)
	{
		m_S_MSG.IOBoard_Cmd |= iMask;
	}
	else
	{
		m_S_MSG.IOBoard_Cmd &= ~iMask;
	}
}

void RelayBoardClient::setSoftEMStop()
{
	m_S_MSG.iSoftEM = 1;
}

void RelayBoardClient::unsetSoftEMStop()
{
	m_S_MSG.iSoftEM = 2;
}

void RelayBoardClient::getIOBoardAnalogIn(int *iAnalogIn)
{
	for (int i = 0; i < 8; i++)
	{
		iAnalogIn[i] = m_REC_MSG.iIOAnalog_In[i];
	}
}

//---------------------------------------------------------------------------------------------------------------------
// USBoard
//---------------------------------------------------------------------------------------------------------------------

bool RelayBoardClient::getUSBoardAvailable()
{
	return m_USBoard.bAvailable;
}

void RelayBoardClient::startUSBoard(int iChannelActive)
{
	m_S_MSG.USBoard_Cmd = iChannelActive;
}

void RelayBoardClient::stopUSBoard()
{
	m_S_MSG.USBoard_Cmd = 0x00;
}

void RelayBoardClient::getUSBoardData1To8(int *iUSDistMM)
{
	for (int i = 0; i < 8; i++)
	{
		iUSDistMM[i] = m_REC_MSG.iUSSensor_Dist[i];
	}
}

void RelayBoardClient::getUSBoardData9To16(int *iUSDistMM)
{
	for (int i = 0; i < 8; i++)
	{
		iUSDistMM[i] = m_REC_MSG.iUSSensor_Dist[i + 8];
	}
}

void RelayBoardClient::getUSBoardAnalogIn(int *iAnalogIn)
{
	for (int i = 0; i < 4; i++)
	{
		iAnalogIn[i] = m_REC_MSG.iUSAnalog_In[i];
	}
}

//---------------------------------------------------------------------------------------------------------------------
// Logging
//---------------------------------------------------------------------------------------------------------------------

void RelayBoardClient::enable_logging()
{
	m_blogging = true;
}

void RelayBoardClient::disable_logging()
{
	m_blogging = false;
}

void RelayBoardClient::log_to_file(int direction, unsigned char cMsg[], int iNumBytes)
{
	// Open Logfile
	FILE *pFile = fopen("neo_relayboard_RX_TX_log.log", "a");

	if (!pFile)
	{
		return;
	}

	// Write Data to Logfile
	fprintf(pFile, "Direction %i =>", direction);
	for (int i = 0; i < iNumBytes; i++)
	{
		fprintf(pFile, " %.2x", cMsg[i]);
	}
	fprintf(pFile, "\n");

	// Close Logfile
	fclose(pFile);
}

//---------------------------------------------------------------------------------------------------------------------
// Data Converter
//---------------------------------------------------------------------------------------------------------------------

void RelayBoardClient::convRecMsgToData(unsigned char cMsg[])
{
	int iCnt = 0;

	// Has Data
	iCnt++;

	// Relayboard Status
	m_REC_MSG.iRelayBoard_Status = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// Charging Current
	m_REC_MSG.iCharging_Current = int16_t((cMsg[iCnt + 1] << 8) | cMsg[iCnt]);
	iCnt += 2;

	// Charging State
	m_REC_MSG.iCharging_State = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// Battery Voltage
	m_REC_MSG.iBattery_Voltage = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// Keypad
	m_REC_MSG.iKey_Pad = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
	iCnt += 2;

	// Temp Sensor
	m_REC_MSG.iTemp_Sensor = int16_t((cMsg[iCnt + 1] << 8) | cMsg[iCnt]);
	iCnt += 2;

	// IOBoard
	if (m_iFoundExtHardware & 0x1)
	{
		m_REC_MSG.iIODig_In = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;

		for (int i = 0; i < 8; i++)
		{
			m_REC_MSG.iIOAnalog_In[i] = int16_t((cMsg[iCnt + 1] << 8) | cMsg[iCnt]);
			iCnt += 2;
		}

		m_REC_MSG.iIOBoard_State = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	// USBoard
	if (m_iFoundExtHardware & 0x2)
	{
		for (int i = 0; i < 16; i++)
		{
			m_REC_MSG.iUSSensor_Dist[i] = cMsg[iCnt++];
		}
		for (int i = 0; i < 4; i++)
		{
			m_REC_MSG.iUSAnalog_In[i] = int16_t((cMsg[iCnt + 1] << 8) | cMsg[iCnt]);
			iCnt += 2;
		}

		m_REC_MSG.iUSBoard_State = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
		iCnt += 2;
	}

	// Motor Data
	for (int i = 0; i < 8; ++i)
	{
		if (m_Motor[i].bActive)
		{
			// Enc Data
			m_Motor[i].lEnc = (cMsg[iCnt + 3] << 24) | (cMsg[iCnt + 2] << 16) | (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 4;
			// EncS Data
			m_Motor[i].lEncS = (cMsg[iCnt + 3] << 24) | (cMsg[iCnt + 2] << 16) | (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 4;
			// Motor Status
			m_Motor[i].iStatus = (cMsg[iCnt + 1] << 8) | cMsg[iCnt];
			iCnt += 2;
		}
	}
}

int RelayBoardClient::convDataToSendMsg(unsigned char cMsg[])
{
	int iCnt = 0;
	int iDataAvailable = 0;
	int bMotoData8 = 0;
	int bMotoData4 = 0;

	if ((m_Motor[0].bActive) || (m_Motor[1].bActive) || (m_Motor[2].bActive) || (m_Motor[3].bActive))
	{
		bMotoData4 = 1;
		if ((m_Motor[4].bActive) || (m_Motor[5].bActive) || (m_Motor[6].bActive) || (m_Motor[7].bActive))
		{
			bMotoData8 = 1;
		}
	}

	m_bIOBoardData = getIOBoardAvailable();			// send data if hardware is available
	m_bUSBoardData = getUSBoardAvailable();			// send data if hardware is available

	// First Bit not in use yet
	iDataAvailable = (bMotoData8 << 6) | (bMotoData4 << 5) | (m_bRelayData << 4) | (m_bLCDData << 3) | (m_bIOBoardData << 2) | (m_bUSBoardData << 1) | (m_bSpeakerData);

	// Data in Message:
	// Header
	cMsg[iCnt++] = 0x02;
	cMsg[iCnt++] = 0xD6;
	cMsg[iCnt++] = 0x80;
	cMsg[iCnt++] = 0x02;
	// has_data
	// soft_em
	// Motor9-6
	// Motor5-2
	// Relaystates
	// LCD Data
	// IO Data
	// US Data
	// Speaker Data
	// Checksum
	cMsg[iCnt++] = iDataAvailable;
	cMsg[iCnt++] = m_S_MSG.iSoftEM; // SoftEM

	if (bMotoData8)
	{
		// Motor 9
		cMsg[iCnt++] = (m_Motor[7].ldesiredEncS >> 24) & 0xFF;
		cMsg[iCnt++] = (m_Motor[7].ldesiredEncS >> 16) & 0xFF;
		cMsg[iCnt++] = (m_Motor[7].ldesiredEncS >> 8) & 0xFF;
		cMsg[iCnt++] = m_Motor[7].ldesiredEncS & 0xFF;
		// Motor 8
		cMsg[iCnt++] = (m_Motor[6].ldesiredEncS >> 24) & 0xFF;
		cMsg[iCnt++] = (m_Motor[6].ldesiredEncS >> 16) & 0xFF;
		cMsg[iCnt++] = (m_Motor[6].ldesiredEncS >> 8) & 0xFF;
		cMsg[iCnt++] = m_Motor[6].ldesiredEncS & 0xFF;
		// Motor 7
		cMsg[iCnt++] = (m_Motor[5].ldesiredEncS >> 24) & 0xFF;
		cMsg[iCnt++] = (m_Motor[5].ldesiredEncS >> 16) & 0xFF;
		cMsg[iCnt++] = (m_Motor[5].ldesiredEncS >> 8) & 0xFF;
		cMsg[iCnt++] = m_Motor[5].ldesiredEncS & 0xFF;
		// Motor 6
		cMsg[iCnt++] = (m_Motor[4].ldesiredEncS >> 24) & 0xFF;
		cMsg[iCnt++] = (m_Motor[4].ldesiredEncS >> 16) & 0xFF;
		cMsg[iCnt++] = (m_Motor[4].ldesiredEncS >> 8) & 0xFF;
		cMsg[iCnt++] = m_Motor[4].ldesiredEncS & 0xFF;
	}

	if (bMotoData4)
	{
		// Motor 5
		cMsg[iCnt++] = (m_Motor[3].ldesiredEncS >> 24) & 0xFF;
		cMsg[iCnt++] = (m_Motor[3].ldesiredEncS >> 16) & 0xFF;
		cMsg[iCnt++] = (m_Motor[3].ldesiredEncS >> 8) & 0xFF;
		cMsg[iCnt++] = m_Motor[3].ldesiredEncS & 0xFF;
		// Motor 4
		cMsg[iCnt++] = (m_Motor[2].ldesiredEncS >> 24) & 0xFF;
		cMsg[iCnt++] = (m_Motor[2].ldesiredEncS >> 16) & 0xFF;
		cMsg[iCnt++] = (m_Motor[2].ldesiredEncS >> 8) & 0xFF;
		cMsg[iCnt++] = m_Motor[2].ldesiredEncS & 0xFF;
		// Motor 3
		cMsg[iCnt++] = (m_Motor[1].ldesiredEncS >> 24) & 0xFF;
		cMsg[iCnt++] = (m_Motor[1].ldesiredEncS >> 16) & 0xFF;
		cMsg[iCnt++] = (m_Motor[1].ldesiredEncS >> 8) & 0xFF;
		cMsg[iCnt++] = m_Motor[1].ldesiredEncS & 0xFF;
		// Motor 2
		cMsg[iCnt++] = (m_Motor[0].ldesiredEncS >> 24) & 0xFF;
		cMsg[iCnt++] = (m_Motor[0].ldesiredEncS >> 16) & 0xFF;
		cMsg[iCnt++] = (m_Motor[0].ldesiredEncS >> 8) & 0xFF;
		cMsg[iCnt++] = m_Motor[0].ldesiredEncS & 0xFF;
	}

	// Relay states
	if (m_bRelayData)
	{
		cMsg[iCnt++] = m_S_MSG.iCmdRelayBoard >> 8;
		cMsg[iCnt++] = m_S_MSG.iCmdRelayBoard;
	}

	// LCD Data
	if (m_bLCDData)
	{
		//ROS_DEBUG("Display: %s", m_S_MSG.LCD_Txt);
		for (int u = 0; u < 20; u++)
		{
			cMsg[iCnt++] = m_S_MSG.LCD_Txt[u];
		}
	}

	// IO Data
	if (m_bIOBoardData)
	{
		cMsg[iCnt++] = m_S_MSG.IOBoard_Cmd >> 8;
		cMsg[iCnt++] = m_S_MSG.IOBoard_Cmd;
	}

	// US Data
	if (m_bUSBoardData)
	{
		cMsg[iCnt++] = m_S_MSG.USBoard_Cmd >> 8;
		cMsg[iCnt++] = m_S_MSG.USBoard_Cmd;
	}

	// Speaker Data
	if (m_bSpeakerData)
	{
		cMsg[iCnt++] = m_S_MSG.Speaker >> 8;
		cMsg[iCnt++] = m_S_MSG.SpeakerLoud;
	}

	const int iNumBytesSend = iCnt;

	// Calc checksum
	int iChkSum = 0;
	for (int i = 4; i < iNumBytesSend; i++)
	{
		iChkSum %= 0xFF00;
		iChkSum += cMsg[i];
	}
	cMsg[iNumBytesSend] = iChkSum >> 8;
	cMsg[iNumBytesSend + 1] = iChkSum;

	// Reset data indicators
	m_bLCDData = false;

	return iNumBytesSend + 2;
}
