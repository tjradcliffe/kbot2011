/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "I2C_Ultrasound.h"
#include "DigitalModule.h"
#include "I2C.h"
#include "Utility.h"
#include "WPIStatus.h"

const UINT8 I2C_Ultrasound::kAddress = 0xE0;
const UINT8 I2C_Ultrasound::kCommandRegister = 0x00;
const UINT8 I2C_Ultrasound::kMaxGainRegister = 0x01;
const UINT8 I2C_Ultrasound::kRangeRegister = 0x02;
const UINT8 I2C_Ultrasound::kRangeHighByteRegister = 0x02;
const UINT8 I2C_Ultrasound::kRangeLowByteRegister = 0x03;	

const UINT8 I2C_Ultrasound::kRangeModeInches = 0x50;
const UINT8 I2C_Ultrasound::kRangeModeCentimeters = 0x51;
const UINT8 I2C_Ultrasound::kRangeModeMicroseconds = 0x52;

const UINT8 I2C_Ultrasound::kChangeAddressStep1 = 0xA0;
const UINT8 I2C_Ultrasound::kChangeAddressStep2 = 0xA5;
const UINT8 I2C_Ultrasound::kChangeAddressStep3 = 0xAA;
	
const UINT8 I2C_Ultrasound::kSetGain40 = 0x01;
const UINT8 I2C_Ultrasound::kSetGain50 = 0x01;
const UINT8 I2C_Ultrasound::kSetGain60 = 0x03;
const UINT8 I2C_Ultrasound::kSetGain70 = 0x04;
const UINT8 I2C_Ultrasound::kSetGain80 = 0x05;
const UINT8 I2C_Ultrasound::kSetGain100 = 0x06;
const UINT8 I2C_Ultrasound::kSetGain120 = 0x07;
const UINT8 I2C_Ultrasound::kSetGain140 = 0x08;
const UINT8 I2C_Ultrasound::kSetGain200 = 0x09;
const UINT8 I2C_Ultrasound::kSetGain250 = 0x0A;
const UINT8 I2C_Ultrasound::kSetGain300 = 0x0B;
const UINT8 I2C_Ultrasound::kSetGain350 = 0x0C;
const UINT8 I2C_Ultrasound::kSetGain400 = 0x0D;
const UINT8 I2C_Ultrasound::kSetGain500 = 0x0E;
const UINT8 I2C_Ultrasound::kSetGain600 = 0x0F;
const UINT8 I2C_Ultrasound::kSetGain700 = 0x10;

/**
 * Constructor.
 * 
 * @param slot The slot of the digital module that the sensor is plugged into.
 */
I2C_Ultrasound::I2C_Ultrasound(UINT32 slot)
	: m_i2c (NULL)
{
	DigitalModule *module = DigitalModule::GetInstance(slot);
	if (module)
	{
		m_i2c = module->GetI2C(kAddress);
	}
}

/**
 * Destructor.
 */
I2C_Ultrasound::~I2C_Ultrasound()
{
	delete m_i2c;
	m_i2c = 0;
}

/**
Return the distance in cm

@return distance in cm
*/
float I2C_Ultrasound::GetDistance()
{
	UINT16 distance = 0;
	UINT8 data = 0;
	if (m_i2c)
	{
		m_i2c->Read(kRangeHighByteRegister, 1, &data);
		distance = data << 8;
		m_i2c->Read(kRangeLowByteRegister, 1, &data);
		distance += data;
	}
	return (float)distance;
}

