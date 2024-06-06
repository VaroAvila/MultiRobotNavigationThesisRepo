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


#ifndef DRIVEPARAM_INCLUDEDEF_H
#define DRIVEPARAM_INCLUDEDEF_H

#include "StrUtil.h"

const double PI = 3.14159265358979323846;

/**
 * Parameters and conversion functionality of a motor drive.
 */
class DriveParam
{
public:
	DriveParam()
	{
        bmotor_active = false;
        bhoming_active = false;
        bmotor_avaliable = false;
        bmotor_homed = false;
		iEncIncrPerRevMot = 0;
		dVelMeasFrqHz = 0;
		dGearRatio = 0;
		dBeltRatio = 0;
		iSign = 0;
		dVelMaxEncIncrS = 0;
		dVelPModeEncIncrS = 0;
		dAccIncrS2 = 0;
		dDecIncrS2 = 0;
		dRadToIncr = 0;
	}

    std::string sName;
    bool bmotor_active;			// if motor is active
    bool bhoming_active;		// if homing is nessecary
    bool bmotor_avaliable;   	// if motor is available
    bool bmotor_homed;       	// if homing was succssesful
	int iEncIncrPerRevMot;		// encoder increments per revolution of motor shaft
	double dVelMeasFrqHz;		// only used for Neo drive = 500, else = 1
	double dGearRatio;			// gear ratio
	double dBeltRatio;			// if drive has belt set ratio, else = 1
	int iSign;					// direction of motion
	double dVelMaxEncIncrS;		// max. veloctiy [encoder increments / s]
	double dVelPModeEncIncrS;	// velocity in position mode e. g. if amplifier generates trajectory
	double dAccIncrS2;			// max. acceleration [encoder increments / S^2]
	double dDecIncrS2;			// max. deceleration [encoder increments / S^2]
	double dModulo;				// Modulo for ELMO encoder
    double dRadToIncr; 			// factor for conversion


	void calcRadToIncr()
	{
		dRadToIncr = (iEncIncrPerRevMot * dGearRatio * dBeltRatio) / (2. * PI);
	}

	/**
	 * Converts position and velocity.
	 * @param dPosRad			position in radians
	 * @param dVelRadS			velocity in radians per seconds
	 * @param piPosIncr			converted position in increments
	 * @param piVelIncrPeriod	converted velocity in increments per period
	 */
	void convRadSToIncrPerPeriod(double dPosRad, double dVelRadS, int* piPosIncr, int* piVelIncrPeriod)
	{
		*piPosIncr = (int)convRadToIncr(dPosRad);
		*piVelIncrPeriod = (int)convRadSToIncrPerPeriod(dVelRadS);
	}

	/// Conversions of wheel angle in radians to encoder increments.
	double convRadToIncr(double dPosWheelRad)
	{
		return (dPosWheelRad * dRadToIncr);
	}

	/// Conversions of encoder increments to wheel angle in radians.
	double convIncrToRad(int iPosIncr)
	{
		return ((double)iPosIncr / dRadToIncr);
	}

	/// Conversions of gear velocity in rad/s to encoder increments per measurement period.
	double convRadSToIncrPerPeriod(double dVelWheelRadS)
	{
		return ( (dVelWheelRadS * dRadToIncr) / dVelMeasFrqHz );
	}

	/// Conversions of encoder increments per measurment period to gear velocity in rad/s.
	double convIncrPerPeriodToRadS(int iVelMotIncrPeriod)
	{
		return ( (double)iVelMotIncrPeriod / dRadToIncr * dVelMeasFrqHz );
	}

};

#endif
