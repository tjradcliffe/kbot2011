#ifndef MAPPINGS_H
#define MAPPINGS_H

//! Button indices (not an enum for type purposes)
const int knMoveToWall = 3;
const int knStrafe = 4;


//! Axis mappings
const int knX = 0;
const int knY = 1;
const int knR = 2;

// The cRIO slots for various interfaces
const int knAnalogSlot = 1;
const int knDigitalSlot = 4;
const int knRelaySlot = 8;

//! The JAG ids for the various motors
enum MotorMappings
{
	knRightBackJaguar = 5,
	knLeftBackJaguar = 2,
	knRightFrontJaguar = 6,
	knLeftFrontJaguar = 3,
	knArmJaguar = 1,
	knLowerRollerJaguar = 4,
	knUpperRollerJaguar = 7,
};

//! The solenoid ids
enum SolenoidMappings
{
	knWristOutSolenoid = 1,
	knWristInSolenoid = 2,
	knJawOpenSolenoid = 3,
	knJawClosedSolenoid = 4,
	knDeployerOutSolenoid = 5,
	knDeployerInSolenoid = 6,
};

//! The analog sensor mappings
enum AnalogMapping
{
	knGyro = 1,
	knLeftIRSensor = 2,
	knLeftUltrasound = 0xE2,
	knRightIRSensor = 3,
	knRightUltrasound = 0xE0,
	knAccelerationX = 4,
	knAccelerationY = 5,
	knAccelerationZ = 6,
	knArmAngle = 7,
	knArmDistance = 8,
};

//! The digital sensor mappings
enum DigitalMapping
{
	knWristInLimit = 1,
	knWristOutLimit = 2,
	knLineRight = 3,
	knLineLeft = 4,
	knLineBack = 5,
	knRetroReflector = 6,
	knArmUpLimit = 7,
	knArmDownLimit = 8,
};

//! The calculation mappings
enum CalculationMapping
{
	knDriverInput,
	knGyroTracking,
	knWallFollower,
};

#endif // MAPPINGS_H
