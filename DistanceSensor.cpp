/*----------------------------------------------------------------------------*/
/* Copyright (c) K-Botics 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "DistanceSensor.h"
#include "AnalogChannel.h"
#include "AnalogModule.h"
//#include "Timer.h"
#include "Utility.h"
#include "Math.h"
#include "WPIStatus.h"

const UINT32 DistanceSensor::kOversampleBits = 10;
const UINT32 DistanceSensor::kAverageBits = 5;
const float DistanceSensor::kSamplesPerSecond = 50.0;
const float DistanceSensor::kAIRRSExponent = -0.9891; // check
const float DistanceSensor::kAIRRSMultiplier = 57.653;// check
const float DistanceSensor::kAIRRSv2Exponent = -1.0188;
const float DistanceSensor::kAIRRSv2Multiplier = 62.664;

/**
 * Initialize the DistanceSensor.
 */
void DistanceSensor::InitDistanceSensor()
{
	m_analog->SetAverageBits(kAverageBits);
	m_analog->SetOversampleBits(kOversampleBits);
	float sampleRate = kSamplesPerSecond * 
		(1 << (kAverageBits + kOversampleBits));
	m_analog->GetModule()->SetSampleRate(sampleRate);
}

/**
 * DistanceSensor constructor given a slot and a channel.
 * 
 * @param slot The cRIO slot for the analog module the DistanceSensor is connected to.
 * @param channel The analog channel the DistanceSensor is connected to.
 */
DistanceSensor::DistanceSensor(UINT32 slot, UINT32 channel)
{
	m_analog = new AnalogChannel(slot, channel);
	m_channelAllocated = true;
	InitDistanceSensor();
}

/**
 * DistanceSensor constructor with only a channel.
 * 
 * Use the default analog module slot.
 * 
 * @param channel The analog channel the DistanceSensor is connected to.
 */
DistanceSensor::DistanceSensor(UINT32 channel)
{
	m_analog = new AnalogChannel(channel);
	m_channelAllocated = true;
	InitDistanceSensor();
}

/**
 * DistanceSensor constructor with a precreated analog channel object.
 * Use this constructor when the analog channel needs to be shared. There
 * is no reference counting when an AnalogChannel is passed to the DistanceSensor.
 * @param channel The AnalogChannel object that the DistanceSensor is connected to.
 */
DistanceSensor::DistanceSensor(AnalogChannel *channel)
{
	m_analog = channel;
	m_channelAllocated = false;
	if (channel == NULL)
	{
		wpi_fatal(NullParameter);
	}
	else
	{
		InitDistanceSensor();
	}
}

DistanceSensor::DistanceSensor(AnalogChannel &channel)
{
	m_analog = &channel;
	m_channelAllocated = false;
	InitDistanceSensor();
}

/**
 * Delete (free) the accumulator and the analog components used for the DistanceSensor.
 */
DistanceSensor::~DistanceSensor()
{
	if (m_channelAllocated)
		delete m_analog;
}

/**
 * Return the voltage value from the analog channel
 * 
 * @return the voltage value. 
 */
float DistanceSensor::GetVoltage( void )
{
	return m_analog->GetAverageVoltage();
}

/**
 * Return the actual distance in centimeters, based on the formula:
 * distance = multiplier * rawValue ^ exponent
 * 
 * @return the current distance reading in centimeters. 
 */
float DistanceSensor::GetDistance( void )
{
	float voltage = m_analog->GetAverageVoltage();
	double scaledValue = 57.653 * pow(voltage, -0.9891);

	return (float)scaledValue;
}


/**
 * Set the DistanceSensor parameters for the best fit curve based on:
 * distance = multiplier * rawValue ^ exponent
 * 
 * Allows the code to work with multiple DistanceSensors.
 * 
 * @param exponent The exponent of the best fit equation
 * @param multiplier The multiplier of the best fit equation
 */
void DistanceSensor::SetBestFitParameters( float exponent, float multiplier )
{
	m_bestFitExponent = exponent;
	m_bestFitMultiplier = multiplier;
}

/**
 * Get the angle in degrees for the PIDSource base object.
 * 
 * @return The angle in degrees.
 */
double DistanceSensor::PIDGet()
{
	return GetDistance();
}
