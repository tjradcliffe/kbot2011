#ifndef MAPPINGS_H
#define MAPPINGS_H

//! Buttons per stick
const int knButtons = 10;

//! Axes per stick
const int knAxes = 5;

//! Button indices (not an enum for type purposes)
const int knMoveToWallButton = 3;
const int knStrafeButton = 4;
const int knArmParkedButton = knButtons+3; // button 4 on operator stick
const int knArmLowButton = knButtons+0; // button 1 on operator stick
const int knArmMiddleButton = knButtons+1; // button 2 on operator stick
const int knArmHighButton = knButtons+2; // button 3 on operator stick
const int knWristInButton = knButtons+4; // not sure
const int knWristOutButton = knButtons+5; // not sure

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

//! Arm states
enum ArmStates
{
	knArmParked,  // wrist in, arm down, all motors stopped
	knArmLow,
	knArmMiddle,
	knArmHigh,
};

//! Wrist states (ignored if arm parked)
enum WristStates
{
	knWristIn,
	knWristOut,
};

//! Minibot deployer states
enum MiniBotStates
{
	knDeployerIn,
	knDeployerOut,
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

//! The relay ids
enum RelayMappings
{
	knCompressorRelay = 1,
};

//! The analog sensor mappings
enum AnalogMapping
{
	knGyro = 1,
	knLeftIRSensor = 2,
	knLeftUltrasound = 0xE2,
	knRightIRSensor = 3,
	knRightUltrasound = 0xE0,
	knArmAngle = 4,
	knTubeIR = 5,
	knAccelerometer = 0x3A,
	knAccelerationX,	// these will just be sequential after 0x3A
	knAccelerationY,
	knAccelerationZ,
};

//! The digital sensor mappings
enum DigitalMapping
{
	knLineRight = 1,
	knLineLeft = 2,
	knLineBack = 3,
	knRetroReflector = 4,
	knTubeLeft = 5,
	knTubeRight = 6,
	knCompressorLimit = 7,
	knRecordSwitch = 8,
};

//! The calculation mappings
enum CalculationMapping
{
	knDriverInput,
	knGyroTracking,
	knWallFollower,
};

#endif // MAPPINGS_H
