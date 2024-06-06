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

#ifndef RelayBoardClient_INCLUDEDEF_H
#define RelayBoardClient_INCLUDEDEF_H

#include "StrUtil.h"
#include "DriveParam.h"


/**
 * Message handler class for communication with a Neobotix RelayBoardV2.
 *
 */
class RelayBoardClient
{
public:
	// Constructor
	RelayBoardClient();

	// Deconstructor
	~RelayBoardClient();

	//------------------------------Initialisation--------------------------------------------------------------------
	int init(const char *device_name, int iactive_motors, int ihoming_motors, int iext_hardware,
			 long lModulo0, long lModulo1, long lModulo2, long lModulo3,
			 long lModulo4, long lModulo5, long lModulo6, long lModulo7);

	bool checkConfig();
	void getConfig(int *iactivated_motors, int *ihomed_motors, int *iavaliable_hardware, int *iconfigured);

	bool shutdownPltf();

	//-------------------------------Msg Handling----------------------------------------------------------------------
	bool waitForRx(uint64_t timeout_usec);

	int evalRxBuffer(uint64_t timeout_usec = 100 * 1000);

	int sendDataToRelayBoard();

	int convDataToSendMsg(unsigned char cMsg[]);

	void convRecMsgToData(unsigned char cMsg[]);

	//------------------------------Set Data for next Message----------------------------------------------------------

	// RelayBoard
	void startCharging();
	void stopCharging();
	void setRelayBoardDigOut(int iChannel, bool bOn);

	// Motors
	void setMotorDesiredEncS(int iMotorNr, long dVel);

	// LCD
	void writeLCD(const std::string &sText);

	// USBoard
	void startUSBoard(int iChannelActive);
	void stopUSBoard();

	// IOBoard
	void setIOBoardDigOut(int iChannel, bool bVal);

	// EM-Stop
	void setSoftEMStop();
	void unsetSoftEMStop();

	//------------------------------Get Data for Topics----------------------------------------------------------------
	// EM-Stop
	bool isEMStop();
	bool isSoftEMStop();
	bool isScannerStop();

	// RelayBoard Data
	void getRelayBoardState(int *state);
	void getTemperature(int *temp);
	void getBattVoltage(int *battvolt);
	void getChargingCurrent(int *current);
	void getChargingState(int *state);
	void getRelayBoardDigOut(int *digOut);
	bool getRelayBoardDigOutState(int ID);
	void getKeyPad(int *keypad);
	bool getKeyState(int ID);

	// Motors
	void getMotorEnc(int iMotorNr, long *lMotorEnc);
	void getMotorEncS(int iMotorNr, long *lMotorEncS);
	void getMotorState(int iMotorNr, int *iMotorStatus);
	bool getMotorAvailable(int iMotorNr);
	bool getMotorHomed(int iMotorNr);

	// USBoard
	bool getUSBoardAvailable();
	void getUSBoardData1To8(int *iUSDistMM);
	void getUSBoardData9To16(int *iUSDistMM);
	void getUSBoardAnalogIn(int *iAnalogIn);

	// IOBoard
	bool getIOBoardAvailable();
	void getIOBoardDigIn(int *digIn);
	bool getIOBoardDigInState(int ID);
	void getIOBoardDigOut(int *digOut);
	bool getIOBoardDigOutState(int ID);
	void getIOBoardAnalogIn(int *iAnalogIn);
	int getIOBoardAnalogInRawValue(int ID);

	//------------------------------Logging----------------------------------------------------------------------------
	void enable_logging();
	void disable_logging();
	void log_to_file(int direction, unsigned char cMsg[], int numBytes);

	enum RelBoardReturns
	{
		NO_ERROR = 0,
		NOT_INITIALIZED = 1,
		GENERAL_SENDING_ERROR = 2,
		TOO_LESS_BYTES_IN_QUEUE = 3,
		NO_MESSAGES = 4, // for a long time, no message have been received, check com port!
		CHECKSUM_ERROR = 5,
		MSG_CONFIG = 6,
		MSG_DATA = 7
	};

	enum TXReturns
	{
		TX_OK,
		TX_WRITE_FAILED,
	};

	enum EvalRXReturns
	{
		RX_UPDATE_MSG,
		RX_CONFIG_MSG,
		RX_ERROR_MSG,
		RX_UNKNOWN_MSG,
		RX_WRONG_CHECKSUM,
		RX_TIMEOUT,
		RX_NO_HEADER,
		RX_UNKNOWN_ERROR,
	};

	enum InitReturns
	{
		INIT_OPEN_PORT_FAILED,
		INIT_WRITE_FAILED,
		INIT_CONFIG_OK,
		INIT_CONFIG_CHANGED,
		INIT_CONFIG_FAILED,
		INIT_UNKNOWN_ERROR,
	};

	enum ChargingStates
	{
		CHS_NOTCHARGING = 0,
		CHS_START_CHARGING = 1,
		CHS_CHARGING = 2,
		CHS_NO_CHARGER = 3,
		CHS_BRAKES_OPEN = 4,
		CHS_EMSTOP = 5,
		CHS_ABORT = 6,
		CHS_FINISHED = 7,
	};

	enum KeyPad
	{
		KEY_INFO = 0,
		KEY_HOME = 1,
		KEY_START = 2,
		KEY_STOP = 3,
		KEY_RELEASE_BRAKE = 4,
		KEY_ON_DEMAND_1 = 5,
		KEY_ON_DEMAND_2 = 6,
		KEY_ON_DEMAND_3 = 7,
	};

	enum Relays
	{
		RELAY_ON_DEMAND_1 = 0,
		RELAY_ON_DEMAND_2 = 1,
		RELAY_ON_DEMAND_3 = 2,
		RELAY_ON_DEMAND_4 = 3,
	};

private:
	int fd; // serial bus

	// Config Bits
	int m_iFoundMotors;
	int m_iHomedMotors;
	int m_iFoundExtHardware;
	int m_iConfigured;
	int m_iNumBytesRec;

	// Data indicators
	bool m_bRelayData;
	bool m_bLCDData;
	bool m_bIOBoardData;
	bool m_bUSBoardData;
	bool m_bSpeakerData;
	bool m_bChargeData;

	struct Motor
	{
		bool bActive;
		bool bAvailable;
		bool bHoming;
		bool bHomed;
		long lEnc;
		long lEncS;
		int iStatus;
		long ldesiredEncS;
	};
	Motor m_Motor[8];

	struct IOBoard
	{
		bool bActive;
		bool bAvailable;
	};
	IOBoard m_IOBoard;

	struct USBoard
	{
		bool bActive;
		bool bAvailable;
	};
	USBoard m_USBoard;

	struct Send_MSG
	{
		int iSoftEM;
		int iCmdRelayBoard;
		char LCD_Txt[20];
		int IOBoard_Cmd;
		int USBoard_Cmd;
		int Speaker;
		int SpeakerLoud;
	};
	Send_MSG m_S_MSG;

	struct Recived_MSG
	{
		// Standard
		int iRelayBoard_Status;
		int iCharging_Current;
		int iCharging_State;
		int iBattery_Voltage;
		int iKey_Pad;
		int iTemp_Sensor;

		// IOBoard
		int iIODig_In;
		int iIOAnalog_In[8];
		int iIOBoard_State;

		// USBoard
		int iUSSensor_Dist[16];
		int iUSAnalog_In[4];
		int iUSBoard_State;
	};
	Recived_MSG m_REC_MSG;

	bool m_blogging;

};

#endif // RelayBoardClient_INCLUDEDEF_H
