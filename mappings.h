#ifndef MAPPINGS_H
#define MAPPINGS_H

//! Buttons per stick
const int knButtons = 10;

//! Axes per stick
const int knAxes = 5;

/*
NOTE: Xbox controller must be on USB 2
      Logitech controller must be on USB 1
     
_BUTTONS:
_XBOX   CODE
_A 		0
_B		1
_X		2
_Y		3
_LB		4
_RB		5
_LSTICK	8
_RSTICK	9

LOGI    CODE
_1		10
_2		11
_3		12
_4		13
_5		14
_6		15
_7		16
_8		17
_9		18
_10		19
NO BUTTONS ON STICK PRESS

AXES

XBOX    CODE
LEFT->  +0
LEFT^   -1
RIGHT-> +3
RIGHT^  -4
LEFTSQUEEZE +2
RIGHTSQUEEZE -2

CANNOT SEE CROSS

LOGITECH  CODE
LEFT->  	+5
LEFT^   	-6
RIGHT-> 	+7
RIGHT^  	-8
CROSS-> 	+9

NOTE: ^ means pushing the button AWAY from you
*/

//! Various button indices (most in enums)
const int knAllLightsButton = 0;	// Green A Button
const int knRedTubeButton = 1;		// Red B button
const int knBlueTubeButton = 2;		// Blue X button
const int knWhiteTubeButton = 3;	// Yellow Y button
const int knLineFollowButton = 4;	// Left front button
const int knStandardTurnButton = 5;	// Right front button
const int knDeployerOutButton = 7;	// Right little button

const int knAutoScoreAxis = 9;	// Left D-button on Logitech

const int knRecordOverride = 18;	// button 9 on operator controller

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
	knRightFrontJaguar = 4,
	knLeftFrontJaguar = 3,
	knRightBackJaguar = 2,
	knLeftBackJaguar = 8,
	knArmJaguar = 5,
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
	knJawClosed = knButtons+5,
	knJawOpen = knButtons+7,
};

//! Minibot deployer states
enum MiniBotStates
{
	knDeployerIn,
	knDeployerOut,
};

//! Light states
enum LightState
{
	knAllLightsOff,
	knBlueLight,
	knRedLight,
	knWhiteLight,
	knAllLightsOn,
	knTubeCaptureSignal,
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
	knCompressorRelay = 4,
	knRedLightRelay = 1,
	knWhiteLightRelay = 2,
	knBlueLightRelay = 3,
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
	knCompressorLimit = 2,

	knRecordSwitch = 8,	    // TOP--CHECKED 0 (light) is playback, 1 is record
	knOneTwoTubeSwitch = 3,	// 2nd--CHECKED 0 (light) is one tube, 1 is two tubes
	knMirrorSwitch = 4,		// 3rd--CHECKED 0 (light) is right of field (mirrored), 1 is left (unmirrored)	
	knRecoverSwitch = 13,	// 4th--0 is recover all recorded files from playback
	knFifthSwitch = 14,	    // 5th--"What is this quintesence of switch?"

	knRetroReflector = 5, 	// (not implemented) (0 = present)
	knTubeLeft = 6,		// CHECKED (0 = tube)
	knTubeRight = 7,	// CHECKED (0 = tube)
	knLineBack = 10,   // (not implemented) (0 = line) 12 V
	knLineLeft = 11,   // (CHECKED) (0 = line)  12 V
	knLineRight = 12,  // (CHECKED) (0 = line)  12 V
};

//! The calculation mappings
enum CalculationMapping
{
	knDriverInput,
	knGyroTracking,
	knLineFollowing,
	knWallAlign,
};

#endif // MAPPINGS_H
