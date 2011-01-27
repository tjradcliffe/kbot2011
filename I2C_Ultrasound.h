/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __I2C_Ultrasound_h__
#define __I2C_Ultrasound_h__

#include "SensorBase.h"

class I2C;

/**
 * Devantech SRF10 Tiny Ultrasonic Ranger.
 * 
 * This class alows access to an Ultrasonic sensor on an I2C bus.
 * 
 * Details on the sensor can be found here:
 *   http://www.
 * 
 */
class I2C_Ultrasound : public SensorBase
{
public:
	explicit I2C_Ultrasound(UINT8 address);
	virtual ~I2C_Ultrasound();
	float GetDistance();
	void Ping();
	void SetRange(UINT8 range);
	void SetMaxGain(UINT8 maxGain);
	void SetI2CAddress(UINT8 address);

	static const UINT8 kMaxRange;
	
	static const UINT8 kSetGain40;
	static const UINT8 kSetGain50;
	static const UINT8 kSetGain60;
	static const UINT8 kSetGain70;
	static const UINT8 kSetGain80;
	static const UINT8 kSetGain100;
	static const UINT8 kSetGain120;
	static const UINT8 kSetGain140;
	static const UINT8 kSetGain200;
	static const UINT8 kSetGain250;
	static const UINT8 kSetGain300;
	static const UINT8 kSetGain350;
	static const UINT8 kSetGain400;
	static const UINT8 kSetGain500;
	static const UINT8 kSetGain600;
	static const UINT8 kSetGain700;

private:
	static const UINT8 kDefaultAddress;
	static const UINT8 kCommandRegister;
	static const UINT8 kMaxGainRegister;
	static const UINT8 kRangeRegister;
	static const UINT8 kRangeHighByteRegister;
	static const UINT8 kRangeLowByteRegister;	

	static const UINT8 kRangeModeInches;
	static const UINT8 kRangeModeCentimeters;
	static const UINT8 kRangeModeMicroseconds;

	static const UINT8 kChangeAddressStep1;
	static const UINT8 kChangeAddressStep2;
	static const UINT8 kChangeAddressStep3;

	I2C* m_i2c;
};

#endif

