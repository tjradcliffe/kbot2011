#ifndef KBOT_H
#define KBOT_H


#define PID_ARM


// local includes
#include "mappings.h"

// Framework includes
#include "Gyro.h"
#include "WPILib.h"

// Standard includes old-style x.h files => cx files)
#include <map>
#include <cmath>

// local forward declarations
class AutonomousController;
class Controller;
//class I2C_Ultrasound;
class DistanceSensor;
class ScoreThreeController;
class TeleopController;

class KbotPID;

/*!
The main robot class.  This is where almost all of the work gets
done in the robot control framework.  The idea in 2011 is to
make this class handled all of the robot control work in a
consistent, sequential form that is as close as possible identical
in autonomous and telepop.

The main clever feature of the design is to have autonomous 
STRICTLY drive the robot via the same joystick controls that the
driver uses.  The main loop for both autonomous and teleop is
therefore identical: get input from the joystick and put it
into the "current control state" vectors of the KBot object--with
the autonomous "joystick" being a pre-programmed sequence, and
the teleop joystick being the real things--and then read all
sensors, update all control logic, and finally controll all
motors and actuators.
*/ 
class KBot : public IterativeRobot
{
	friend class ScoreThreeController;
	
public:
	
	//! Constructor builds the essentials
	KBot(void);
	
	//! Framework method to initialize robot
	void RobotInit();

	//! Framework method called at the start of disabled
	void DisabledInit();

	//! Framework method called at the start of autonomus
	void AutonomousInit();

	//! Framework method called at the start of teleop
	void TeleopInit();
	
	//! Framework method called on autonomous clock
	void DisabledPeriodic();

	//! Framework method called on autonomous clock
	void AutonomousPeriodic();

	//! Framework method called on teleop clock
	void TeleopPeriodic();

protected:
	
	//! Handle compressor functions
	void ControlCompressor(void);

	//! Reset the whole robot
	void ResetRobot(bool bRecordTeleop = false);
	
	//! Run the robot based on controller state
	void RunRobot(Controller* pController);
	
	//! Apply deadband and normalize all wheel speeds so max is 1.0
	void DeadbandNormalize(double *pfWheelSpeeds);

	//! Rotate a vecotr (fX, fY) through angle fAngle degrees
	void RotateVector(double &fY, double &fX, double fAngle);

	//! Read all the sensors into robot buffers
	void ReadSensors();

	//! Read the ultrasounds (takes some extra logic for cross-talk etc)
	void ReadUltrasoundSensors();
	
	//! Compute actuator inputs
	void ComputeActuators(Controller* pController);
	
	// Compute controller inputs and weight
	void ComputeControllerXYR(Controller* pController);
	
	// Compute gyro inputs and weight
	void ComputeGyroXYR();

	// Compute rotation etc for line following
	void ComputeLineAndWallXYR();
	
	// Compute the arm and deployer actuator outputs
	void ComputeArmAndDeployer(Controller* pController);
	
	// Compute the light states
	void ComputeLights(Controller* pController);
	
	//! Compute the weight given each of the inputs
	void ComputeWeights(Controller* pController);
	
	//! Actually update the actuators
	void UpdateActuators();

	//! Update the wheel speeds
	void UpdateMotors();
	
	//! Update the wrist position
	void UpdateWrist();
	
	//! Update the roller claw jaw position
	void UpdateJaw();
	
	//! Update the deployer position
	void UpdateDeployer();
	
	//! Update the driver station (extended IO)
	void UpdateDriverStation();
	
	//! Update the light relays
	void UpdateLights();
	
protected:
	static const int kPeriodicSpeed;
	
	// PID constants for asymmetric PID controller
	static const float k_posP;
	static const float k_posI;
	static const float k_posD;
	static const float k_negP;
	static const float k_negI;
	static const float k_negD;
	
	static const float kArmGain;
	//! Estimated robot position, orienation and velocity
	std::vector<float> m_vecPosition;
	float m_fOrientation; // positive CCW, 0->360
	std::vector<float> m_vecVelocity;

	//! Digital sensor values
	std::map<DigitalMapping, int> m_mapDigitalSensors;
	
	//! Analog sensor values
	std::map<AnalogMapping, float> m_mapAnalogSensors;
	
	//! Body update values from various sources
	//@{
	std::map<CalculationMapping, float> m_mapX;
	std::map<CalculationMapping, float> m_mapY;
	std::map<CalculationMapping, float> m_mapR;
	//@}
	
	//! Weight vector (appliesto x/y/r from each source)
	//@{
	std::map<CalculationMapping, float> m_mapWeightX;
	std::map<CalculationMapping, float> m_mapWeightY;
	std::map<CalculationMapping, float> m_mapWeightR;
	//@}

	//! arm angle and speed we want
	float m_fTargetArmAngle;
	float m_fArmSpeed;
	
#ifdef PID_ARM
	// Arm PID
	KbotPID *m_pArmPID;
#endif
	
	// Line following PID
	KbotPID *m_pLinePID;
	
	// Line following counter
	int m_nLineCount;
	
	//! wrist position we want (0 == in, 1 == out)
	int m_nWristPosition;
	
	//! jaw position we want (1 == open, 0 == closed)
	int m_nJawPosition;
	
	//! mini-bot deployer position we want (1 == out, 0 == in)
	int m_nDeployerPosition;
	int m_nReleasePosition;
	
	//! jaw roller speeds
	float m_fLowerJawRollerSpeed;
	float m_fUpperJawRollerSpeed;

	// Light state
	LightState m_nLightState;
	Relay	*m_pBlueLightRelay;
	Relay	*m_pRedLightRelay;
	Relay	*m_pWhiteLightRelay;

	// Light servos
	Servo	*m_pBlueLightServo;
	Servo	*m_pRedLightServo;
	Servo	*m_pWhiteLightServo;
	
	// Minibot servos
	Servo	*m_pDeployMinibotArmServo;
	Servo	*m_pReleaseMinibotServo;	
	
	///*************ACTUATORS******************

	void BuildJags();
	void InitJags();
	
	//! Motor controllers for body
	CANJaguar *m_pLeftFrontJaguar;
	CANJaguar *m_pLeftBackJaguar;
	CANJaguar *m_pRightFrontJaguar;
	CANJaguar *m_pRightBackJaguar;
	
	// convenience array of jags
	std::vector<CANJaguar*> m_vecJags;
	std::vector<int> m_vecJagErrors;
	
	//! Motor controller for arm and roller claws
	CANJaguar *m_pArmJaguar;
	CANJaguar *m_pLowerRollerJaguar;
	CANJaguar *m_pUpperRollerJaguar;
	
	//! Solenoids to control wrist, jaw and swing-arm
	Solenoid *m_pWristOutSolenoid;
	Solenoid *m_pWristInSolenoid;
	Solenoid *m_pJawOpenSolenoid;
	Solenoid *m_pJawClosedSolenoid;
	Solenoid *m_pDeployerOutSolenoid;
	Solenoid *m_pDeployerInSolenoid;
	
	///*************SENSORS******************
	// The gyro is used for maintaining orientation
	Gyro *m_pGyro;
	float m_fGyroSetPoint;
	
	// I2C ultrasound sensors
	//I2C_Ultrasound* m_pLeftUltrasound;
	//I2C_Ultrasound* m_pRightUltrasound;
	
	// Stopped count on line
	int m_nStoppedCount;
	
	// Analog distance sensors
	DistanceSensor* m_pLeftIRSensor;
	DistanceSensor* m_pRightIRSensor;
	
	// Arm angle sensor
	AnalogChannel* m_pArmAngle;
	
	// Declare variables for the controllers
	TeleopController *m_pTeleopController;
	AutonomousController *m_pPlaybackController;
	ScoreThreeController *m_pScoreThreeController;

	// compressor control and sensor
	Relay	*m_pCompressorRelay;
	DigitalInput *m_pCompressorLimit;
	
	// line sensors
	DigitalInput* m_pLineRight;
	DigitalInput* m_pLineLeft;
	DigitalInput* m_pLineBack;
	
	// tube sensors
	DigitalInput* m_pTubeLeft;
	DigitalInput* m_pTubeRight;
	AnalogChannel* m_pTubeIR;
	
	// retro-reflector (if we use it)
	DigitalInput* m_pRetroReflector;
	
	// Switches
	DigitalInput* m_pRecordSwitch;		// 0 = record
	DigitalInput* m_pRecoverSwitch;		// 0 = recover
	DigitalInput* m_pMirrorSwitch;		// 0 = invert recorded rotations
	DigitalInput* m_pOneTwoTubeSwitch;	// 0 = one tube, 1 = two tubes
	DigitalInput* m_pFifthSwitch;		// "What is this quintesence of switch?"
	
	// accelerometer
	//ADXL345_I2C* m_pAccelerometer;
	
	int m_autoMode;	// not used
	
	bool m_bRecordOverride; // if true over-rides record/playback switch to record
	bool m_bPreviousOverride; // true if on the last call to DisabledPeriodic we had an override
	
}; // class declaration

#endif // KBOT_H
