#ifndef MAPPINGS_H
#define MAPPINGS_H

//! Button indices (not an enum for type purposes)
const int knMoveToWall = 3;
const int knStrafe = 4;


//! Axis mappings
const int knX = 0;
const int knY = 1;
const int knR = 2;

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
