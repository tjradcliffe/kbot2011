#ifndef MAPPINGS_H
#define MAPPINGS_H

//! Buttons per stick
const int knButtons = 10;

//! Axes per stick
const int knAxes = 5;

/*
NOTE: Xbox controller must be on USB 2
      Logitech controller must be on USB 1
      
XBOX 
*/

//! Various button indices (most in enums)
const int knMoveToWallButton = 3;
const int knStrafeButton = 4;
const int knDeployerInButton = 6;
const int knDeployerOutButton = 7;

//! Axis mappings
const int knX = 0;
const int knY = 1;
const int knR = 2;
const int knRollAround = 7;
const int knRollInOut = 8;
const int knArmUpDown = 6;

// The cRIO slots for various interfaces
const int knAnalogSlot = 1;
const int knDigitalSlot = 4;
const int knRelaySlot = 8;

//! The JAG ids for the various motors
enum MotorMappings
{
	knRightFrontJaguar = 1,
	knLeftFrontJaguar = 2,
	knRightBackJaguar = 3,
	knLeftBackJaguar = 4,
	knArmJaguar = 8,
	knLowerRollerJaguar = 7,
	knUpperRollerJaguar = 6,
};

//! Arm states
enum ArmStates
{
	knArmParked = knButtons,  // wrist in, arm down, all motors stopped
	knArmLow = knButtons+1,
	knArmMiddle = knButtons+2,
	knArmHigh = knButtons+3,
};

//! Wrist & Jaw states (ignored if arm parked)
enum WristJawStates
{
	knWristIn = knButtons+4,
	knWristOut = knButtons+6,
	knJawOpen = knButtons+5,
	knJawClosed = knButtons+7,
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
	knWristInSolenoid = 1,
	knWristOutSolenoid = 2,
	knJawClosedSolenoid = 3,
	knJawOpenSolenoid = 4,
	knDeployerOutSolenoid = 5,
	knDeployerInSolenoid = 6,
};

//! The relay ids
enum RelayMappings
{
	knCompressorRelay = 1,  // ???
	knRedLight = 2,  // ???
	knWhiteLight = 3,  // ???
	knBlueLight = 4,  // ???
};

//! The analog sensor mappings
enum AnalogMapping
{
	knGyroTemp = 1,
	knGyro = 2,
	knArmAngle = 3,
	knTubeIR = 4,
	knLeftIRSensor = 5,
	knRightIRSensor = 6,
	knLeftUltrasound = 0xE2,
	knRightUltrasound = 0xE0,
	knAccelerometer = 0x3A,
	knAccelerationX,	// these will just be sequential after 0x3A
	knAccelerationY,
	knAccelerationZ,
};

//! The digital sensor mappings
enum DigitalMapping
{
	knCompressorLimit = 1,
	knTubeRight = 2,	// CHECKED (0 = tube)
	knTubeLeft = 3,		// CHECKED (0 = tube)
	knLineRight = 6,  // CHECKED (1 = line)
	knLineLeft = 5,   // CHECKED (1 = line)
	knLineBack = 4,   // CHECKED (1 = line)
	knRetroReflector = 7, // CHECKED (0 = present)
	knRecordSwitch = 8,
	knRecoverSwitch = 9,
};

//! The calculation mappings
enum CalculationMapping
{
	knDriverInput,
	knGyroTracking,
	knWallFollower,
};

#endif // MAPPINGS_H
