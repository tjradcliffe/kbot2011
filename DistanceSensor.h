/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef DISTANCE_SENSOR_H_
#define DISTANCE_SENSOR_H_

#include "SensorBase.h"
#include "PIDSource.h"

class AnalogChannel;
class AnalogModule;

/**
 * An analog distance sensor.
 * For example, the AIRRS, Analog Infra-Red Ranging Sensor.
 */
class DistanceSensor : public SensorBase, public PIDSource
{
public:
	static const UINT32 kOversampleBits;
	static const UINT32 kAverageBits;
	static const float kSamplesPerSecond;
	static const float kAIRRSExponent;
	static const float kAIRRSMultiplier;
	static const float kAIRRSv2Exponent;
	static const float kAIRRSv2Multiplier;

	DistanceSensor(UINT32 slot, UINT32 channel);
	explicit DistanceSensor(UINT32 channel);
	explicit DistanceSensor(AnalogChannel *channel);
	explicit DistanceSensor(AnalogChannel &channel);
	virtual ~DistanceSensor();
	float GetVoltage();
	float GetDistance();
	void SetBestFitParameters(float exponent, float multiplier);
	
	double PIDGet();

private:
	void InitDistanceSensor();

	AnalogChannel *m_analog;
	bool m_channelAllocated;
	float m_bestFitExponent;
	float m_bestFitMultiplier;
};
#endif
