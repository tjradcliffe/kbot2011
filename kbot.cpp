
#include "kbot.h"

// local includes
#include "autonomous_controller.h"
#include "constants.h"
#include "DistanceSensor.h"
#include "I2C_Ultrasound.h"
#include "teleop_controller.h"

// FRC includes
#include "I2C.h"

// standard includes
#include <numeric>

const int KBot::kPeriodicSpeed = 50; // Speed of periodic loops in Hz

/*!
The constructor builds everything
*/
KBot::KBot(void)
{
	std::cerr << "In Constructor" << std::endl;
	IterativeRobot::SetPeriod(1.0/kPeriodicSpeed);
	std::cerr << "Periodic rate ="<< IterativeRobot::GetLoopsPerSec() << " loops per second." << std::endl;
	
	// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
	m_pLeftFrontJaguar = new CANJaguar(knLeftFrontJaguar, CANJaguar::kSpeed);
	m_pLeftFrontJaguar->Set(0.0);
	m_pLeftBackJaguar = new CANJaguar(knLeftBackJaguar, CANJaguar::kSpeed);
	m_pLeftBackJaguar->Set(0.0);

	m_pRightFrontJaguar = new CANJaguar(knRightFrontJaguar, CANJaguar::kSpeed);
	m_pRightFrontJaguar->Set(0.0);
	m_pRightBackJaguar = new CANJaguar(knRightBackJaguar, CANJaguar::kSpeed);
	m_pRightBackJaguar->Set(0.0);
	
	// controllers
	m_pTeleopController = new TeleopController("test.dat");
	m_pAutonomousController = new AutonomousController(this, "test.dat");
	
	// gyro
	m_pGyro = new Gyro(knGyro);
	m_fGyroSetPoint = 0.0f;

	// ultrasounds
	m_pLeftUltrasound = new I2C_Ultrasound(knLeftUltrasound);
//	m_pRightUltrasound = new I2C_Ultrasound(knRightUltrasound);
	
	// analog distance sensors
	m_pLeftDistanceSensor = new DistanceSensor(knLeftDistanceSensor);
//	m_pRightDistanceSensor = new DistanceSensor(knRightIRSensor);

}

/*!
This does a fairly hard reset on everything we can get our
hands on.
*/
void KBot::ResetRobot(bool bRecordTeleop)
{
	GetWatchdog().SetEnabled(true);

	m_pGyro->Reset();	// reset gyro and setpoint
	m_fGyroSetPoint = m_pGyro->GetAngle();
	
	if (bRecordTeleop)
	{
		m_pTeleopController->Reset();
	}
	else
	{
		m_pTeleopController->Done();		
	}
	m_pAutonomousController->Reset();
}

/*!
This is the initialization code that gets called when the robot
is created.  Currently it just calls ResetRobot()
*/
void KBot::RobotInit()
{
	GetWatchdog().SetEnabled(true);
	
	m_pLeftFrontJaguar->SetSafetyEnabled(true);
	m_pLeftBackJaguar->SetSafetyEnabled(true);
	m_pRightFrontJaguar->SetSafetyEnabled(true);
	m_pRightBackJaguar->SetSafetyEnabled(true);
	
	m_pLeftFrontJaguar->SetPID(1.0, 0.0, 0.0);
	m_pLeftBackJaguar->SetPID(1.0, 0.0, 0.0);
	m_pRightFrontJaguar->SetPID(1.0, 0.0, 0.0);
	m_pRightBackJaguar->SetPID(1.0, 0.0, 0.0);
	
	m_pLeftFrontJaguar->ConfigEncoderCodesPerRev(360);
	m_pLeftBackJaguar->ConfigEncoderCodesPerRev(360);
	m_pRightFrontJaguar->ConfigEncoderCodesPerRev(360);
	m_pRightBackJaguar->ConfigEncoderCodesPerRev(360);
	
	m_pLeftFrontJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pLeftBackJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pRightFrontJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pRightBackJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			
	m_pLeftFrontJaguar->EnableControl();
	m_pLeftBackJaguar->EnableControl();
	m_pRightFrontJaguar->EnableControl();
	m_pRightBackJaguar->EnableControl();
	
	m_mapAnalogSensors[knGyro] = m_pGyro->GetAngle();
	
	m_pLeftUltrasound->SetRange(I2C_Ultrasound::kMaxRange);
	m_pLeftUltrasound->SetMaxGain(I2C_Ultrasound::kSetGain350);
	
	// Reprogram Ultrasound's I2C address:
	//   Create the Ultrasound with an address of E0 (for an unprogrammed Ultrasound)
	//   then call this method to reprogram it.
	//m_pUltrasound->SetI2CAddress(0xe2);
	
	m_pLeftDistanceSensor->SetBestFitParameters(DistanceSensor::kAIRRSv2Exponent, DistanceSensor::kAIRRSv2Multiplier);
//	m_pRightDistanceSensor->SetBestFitParameters(DistanceSensor::kAIRRSv2Exponent, DistanceSensor::kAIRRSv2Multiplier);
}

/*!
Called at start of disabled
*/
void KBot::DisabledInit() 
{
	// do NOT reset robot here!
}

/*!
Called at start of autonomous
*/
void KBot::AutonomousInit() 
{
	ResetRobot();
}

/*!
Note that for teleop we may want to preserve some information
here, so a full reset may not be what we need.
*/
void KBot::TeleopInit() 
{
	ResetRobot(true);  // true so we get new file for recording
}

/********************************** Periodic Routines *************************************/

void KBot::DisabledPeriodic(void)  
{
	static int nCount = 0;
	
	// feed the user watchdog at every period when disabled
	GetWatchdog().Feed();

	if (1 == nCount)
	{
		m_pLeftUltrasound->Ping();
		std::cerr <<m_pLeftDistanceSensor->GetVoltage() << "   " << m_pLeftDistanceSensor->GetDistance() << "   ";
	}
	else if (14 == nCount)
	{
		std::cerr << m_pLeftUltrasound->GetDistance() << std::endl;
	}
	else if (30 == nCount)
	{
		nCount = 0;
	}
	++nCount;
}

/**
Called on a clock during autonomous
 */
void KBot::AutonomousPeriodic(void)
{
	GetWatchdog().Feed();

	RunRobot(m_pAutonomousController);
}

/**
 Called on a clock during teleop
 */
void KBot::TeleopPeriodic(void) 
{
	//std::cerr << "operator control" << std::endl;
	GetWatchdog().Feed();
	
	RunRobot(m_pTeleopController);
}

void KBot::RunRobot(Controller* pController)
{
	ReadSensors();			// read all the sensors into robot buffers
	pController->Update();	// update the controller buffers from hardware
	ComputeActuators(pController);		// compute contributions to actuator actions
	ComputeWeights(pController);	// compute the weights for each input
	UpdateActuators();		// set the motor and actuator states	
}

/*!
Read or ping the ultrasounds, depending on where we are
in an internal loop.  They are pinged at different times,
which may create issues with the control logic.
*/
void KBot::ReadUltrasoundSensors()
{
	static int nUltrasoundCount = 0;
	
	if (1 == nUltrasoundCount)
	{
		m_pLeftUltrasound->Ping();
	}
	else if (5 == nUltrasoundCount)
	{
		float fDistance = m_pLeftUltrasound->GetDistance();
		if (10.0 > fDistance)
		{
			m_mapAnalogSensors[knLeftUltrasound] = fDistance;
		}
//		m_pRightUltrasound->Ping();
	}
	else if (10 == nUltrasoundCount)
	{
		nUltrasoundCount = 0;
//		float fDistance = m_pRightUltrasound->GetDistance();
//		if (10.0 > fDistance)
//		{
//			m_mapAnalogSensors[knRightUltrasound] = fDistance;
//		}
	}
	++nUltrasoundCount;
	
}

/*!
Read all the sensors, at least conceptually.  Some of them,
like the ultrasounds, don't get read every time due to 
ping delays.
*/
void KBot::ReadSensors()
{
	//*********ANALOG SENSORS************
	
	// read current gyro angle
	m_mapAnalogSensors[knGyro] = m_pGyro->GetAngle();
	
	m_mapAnalogSensors[knLeftDistanceSensor] = m_pLeftDistanceSensor->GetDistance();
//	m_mapAnalogSensors[knRightDistanceSensor] = m_pRightDistanceSensor->GetDistance();
	
	ReadUltrasoundSensors();	// put ping logic in its own method
	
//	m_mapAnalogSensors[knArmPosition] = m_pArm->GetValue();

	//*********DIGITAL SENSORS************
	
	// Wrist limit switches
	//m_mapDigitalSensors[knWristInSwitch] = m_pWristIn->GetValue();
	//m_mapDigitalSensors[knWristOutSwitch] = m_pWristOut->GetValue();
	
}

/*!
Compute the XYR for the controller.  If we have a program
button for "move-to-wall", say, this is where it will be
implemented for computing the desired action, rather than
using the stick values (which is the default)
*/
void KBot::ComputeControllerXYR(Controller* pController)
{
	m_mapX[knDriverInput] = -pController->GetAxis(knX);
	m_mapY[knDriverInput]  = -pController->GetAxis(knY);
	m_mapR[knDriverInput] = pController->GetAxis(knR);	
	if (pController->GetButton(knMoveToWall))
	{
		// implement move-to-wall logic based on sensors here
	}
	else if (pController->GetButton(knStrafe))
	{
		// implement strafing logic here
	}
}

/*!
Compute the rotation we want based on gyroscopic station-keeping
*/
void KBot::ComputeGyroXYR()
{
	m_mapR[knGyroTracking] = kfRotationFactor*(m_mapAnalogSensors[knGyro]-m_fGyroSetPoint);	
}

/*!
Compute what we want the robot to do in terms of XYR for the
main body and angle, wrist and roller state for the arm.
*/
void KBot::ComputeActuators(Controller* pController)
{
	ComputeControllerXYR(pController);
	ComputeGyroXYR();
}

/*!
This is where the other half of the modal logic lives.  Some of
it is in the ComputeControllerXYR() based on what buttons are 
pushed, and some of it is here, based on the same thing.  The
reason for splitting these two aspects is that we have all
the information we need at this point to make true modal decisions,
some of which may require us to over-ride other decsions here.
*/
void KBot::ComputeWeights(Controller* pController)
{
	m_mapWeightX[knDriverInput] = 1.0f;	// assume driver full control
	m_mapWeightY[knDriverInput] = 1.0f;
	m_mapWeightR[knDriverInput] = 1.0f;

	m_mapWeightX[knGyroTracking] = 0.0f; // assume no input from gyro
	m_mapWeightY[knGyroTracking] = 0.0f;
	m_mapWeightR[knGyroTracking] = 0.0f;
	
	if (fabs(m_mapR[knDriverInput]) > 0.1f)	// let stick have control
	{
		m_fGyroSetPoint = m_mapAnalogSensors[knGyro];
		m_mapWeightR[knGyroTracking] = 0.0f;
	}
	else						// let gyro have control
	{
		m_mapWeightR[knGyroTracking] = 1.0f;
		m_mapWeightR[knDriverInput] = 0.0f;
	}	
}

void KBot::UpdateWheelSpeeds()
{
	float fX = 0.0f;	// we will eventually add arm 
	float fY = 0.0f;	// and other values here
	float fR = 0.0f;
	// rather awkwardly iterate over all weights and inputs for motors
	std::map<CalculationMapping, float>::iterator itWeights = m_mapWeightX.begin();
	for(; itWeights != m_mapWeightX.end(); ++itWeights)
	{
		CalculationMapping nIndex = itWeights->first;
		fX += m_mapWeightX[nIndex]*m_mapX[nIndex];
		fY += m_mapWeightY[nIndex]*m_mapY[nIndex];
		fR += m_mapWeightR[nIndex]*m_mapR[nIndex];
	}
	
	double wheelSpeeds[4];
	wheelSpeeds[0] = fX + fY + fR;
	wheelSpeeds[1] = -fX + fY - fR;
	wheelSpeeds[2] = -fX + fY + fR;
	wheelSpeeds[3] = fX + fY - fR;
	
	DeadbandNormalize(wheelSpeeds);

	// actually set speeds
	UINT8 syncGroup = 0x80;	
	m_pLeftFrontJaguar->Set(wheelSpeeds[0]*100.0 , syncGroup);
	m_pRightFrontJaguar->Set(wheelSpeeds[1]*100.0 , syncGroup);
	m_pRightBackJaguar->Set(wheelSpeeds[2]*100.0, syncGroup);
	m_pLeftBackJaguar->Set(wheelSpeeds[3]*100.0 , syncGroup);
	CANJaguar::UpdateSyncGroup(syncGroup);
}

void KBot::UpdateArmPosition()
{
	
}

void KBot::UpdateRollerClaw()
{
	
}

void KBot::UpdateActuators()
{
	UpdateWheelSpeeds();
	UpdateArmPosition();
	UpdateRollerClaw();
}

/**
If a wheel speed is less than some limit zero it, and 
then normalize all speeds to the max so the max speed is 1.0
 */
void KBot::DeadbandNormalize(double *wheelSpeeds)
{
	// Deadband on wheels:
	for (int i=0; i<4; i++)
	{
		if (fabs(wheelSpeeds[i])<0.05)
		{
			wheelSpeeds[i] = 0.0;
		}
	}
	
	double maxMagnitude = fabs(wheelSpeeds[0]);
	INT32 i;
	for (i=1; i<4; i++)
	{
		double temp = fabs(wheelSpeeds[i]);
		if (maxMagnitude < temp) maxMagnitude = temp;
	}
	if (maxMagnitude > 1.0)
	{
		for (i=0; i<4; i++)
		{
			wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
		}
	}
}

/**
 * Rotate a vector in Cartesian space.
 */
void KBot::RotateVector(double &x, double &y, double angle)
{
	double cosA = cos(angle * (3.14159 / 180.0));
	double sinA = sin(angle * (3.14159 / 180.0));
	double xOut = x * cosA - y * sinA;
	double yOut = x * sinA + y * cosA;
	x = xOut;
	y = yOut;
}

START_ROBOT_CLASS(KBot);

