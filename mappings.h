#ifndef MAPPINGS_H
#define MAPPINGS_H

//! The JAG ids for the various motors
enum MotorMappings
{
	knRightBackJaguar = 5,
	knLeftBackJaguar = 2,
	knRightFrontJaguar = 6,
	knLeftFrontJaguar = 3,
};

//! The analog sensor mappings
enum AnalogMapping
{
	knGyro = 1,
	knLeftDistanceSensor = 2,
	knLeftUltrasound = 0xE2,
};

//! The digital sensor mappings
enum DigitalMapping
{
};

//! The calculation mappings
enum CalculationMapping
{
	knDriverInput,
	knGyroTracking,
	knWallFollowers,
};

#endif // MAPPINGS_H
