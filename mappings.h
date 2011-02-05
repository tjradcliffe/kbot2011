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
	knArmJaguar = 1,
	knLowerRollerJaguar = 4,
	knRightRollerJaguar = 7,
};

//! The solenoid ids
enum SolenoidMappings
{
	knWristOutSolenoid = 1,
	knWristInSolenoid = 2,
	knJawOpenSolenoid = 3,
	knJawClosedSolenoid = 4,
	knDeployerSolenoid = 5,
};

//! The analog sensor mappings
enum AnalogMapping
{
	knGyro = 1,
	knLeftDistanceSensor = 2,
	knLeftUltrasound = 0xE2,
	knRightDistanceSensor = 3,
	knRightUltrasound = 0xE3,
	knAccelerationX = 4,
	knAccelerationY = 5,
	knAccelerationZ = 6,
	knArmAngle = 7,
};

//! The digital sensor mappings
enum DigitalMapping
{
	knWristIn = 1,
	knWristOut = 2,
	knLineRight = 3,
	knLineLeft = 4,
	knLineBack = 5,
	knRetroReflector = 6,
	knArmUp = 7,
	knArmDown = 8,
};

//! The calculation mappings
enum CalculationMapping
{
	knDriverInput,
	knGyroTracking,
	knWallFollowers,
};

#endif // MAPPINGS_H
