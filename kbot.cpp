
#include "kbot.h"

// local includes
#include "autonomous_controller.h"
#include "I2C_Ultrasound.h"
#include "DistanceSensor.h"
#include "teleop_controller.h"

// FRC includes
#include "I2C.h"

// standard includes
#include <numeric>

const int KBot::kPeriodicSpeed = 200; // Speed of periodic loops in Hz

/*!
The constructor builds everything
*/
KBot::KBot(void)
{
	std::cerr << "In Constructor" << std::endl;
	IterativeRobot::SetPeriod(1.0/kPeriodicSpeed);
	std::cerr << "Periodic rate ="<< IterativeRobot::GetLoopsPerSec() << " loops per second." << std::endl;
	
	// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
	m_pLeftJaguarFront = new CANJaguar(3, CANJaguar::kSpeed);
	m_pLeftJaguarFront->Set(0.0);
	m_pLeftJaguarBack = new CANJaguar(2, CANJaguar::kSpeed);
	m_pLeftJaguarBack->Set(0.0);

	m_pRightJaguarFront = new CANJaguar(6, CANJaguar::kSpeed);
	m_pRightJaguarFront->Set(0.0);
	m_pRightJaguarBack = new CANJaguar(5, CANJaguar::kSpeed);
	m_pRightJaguarBack->Set(0.0);
	
	// controllers
	m_pTeleopController = new TeleopController("test.dat");
	m_pAutonomousController = new AutonomousController(this, "test.dat");
	
	// gyro
	m_pGyro = new Gyro(1);
	m_fGyroSetPoint = 0.0f;

	// ultrasounds
	m_pUltrasound = new I2C_Ultrasound(0xE2);
	
	// analog distance sensors
	m_pDistanceSensor = new DistanceSensor(2);
	
	m_vecX.resize(1);
	m_vecY.resize(1);
	m_vecR.resize(1);
	
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
	
	m_nAnalogSensorNumber = 1;
	m_vecAnalogSensors.resize(m_nAnalogSensorNumber);
	
	m_pLeftJaguarFront->SetSafetyEnabled(true);
	m_pLeftJaguarBack->SetSafetyEnabled(true);
	m_pRightJaguarFront->SetSafetyEnabled(true);
	m_pRightJaguarBack->SetSafetyEnabled(true);
	
	m_pLeftJaguarFront->SetPID(1.0, 0.0, 0.0);
	m_pLeftJaguarBack->SetPID(1.0, 0.0, 0.0);
	m_pRightJaguarFront->SetPID(1.0, 0.0, 0.0);
	m_pRightJaguarBack->SetPID(1.0, 0.0, 0.0);
	
	m_pLeftJaguarFront->ConfigEncoderCodesPerRev(360);
	m_pLeftJaguarBack->ConfigEncoderCodesPerRev(360);
	m_pRightJaguarFront->ConfigEncoderCodesPerRev(360);
	m_pRightJaguarBack->ConfigEncoderCodesPerRev(360);
	
	//m_pLeftJaguarFront->ConfigMaxOutputVoltage(6.0);
	//m_pLeftJaguarBack->ConfigMaxOutputVoltage(6.0);
	//m_pRightJaguarFront->ConfigMaxOutputVoltage(6.0);
	//m_pRightJaguarBack->ConfigMaxOutputVoltage(6.0);
	
	m_pLeftJaguarFront->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pLeftJaguarBack->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pRightJaguarFront->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pRightJaguarBack->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
			
	m_pLeftJaguarFront->EnableControl();
	m_pLeftJaguarBack->EnableControl();
	m_pRightJaguarFront->EnableControl();
	m_pRightJaguarBack->EnableControl();
	
	m_vecAnalogSensors[GYRO] = m_pGyro->GetAngle();
	
	m_pUltrasound->SetRange(I2C_Ultrasound::kMaxRange);
	m_pUltrasound->SetMaxGain(I2C_Ultrasound::kSetGain350);
	
	// Reprogram Ultrasound's I2C address:
	//   Create the Ultrasound with an address of E0 (for an unprogrammed Ultrasound)
	//   then call this method to reprogram it.
	//m_pUltrasound->SetI2CAddress(0xe2);
	
	m_pDistanceSensor->SetBestFitParameters(DistanceSensor::kAIRRSv2Exponent, DistanceSensor::kAIRRSv2Multiplier);
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
		m_pUltrasound->Ping();
		std::cerr <<m_pDistanceSensor->GetVoltage() << "   " << m_pDistanceSensor->GetDistance() << "   ";
	}
	else if (14 == nCount)
	{
		std::cerr << m_pUltrasound->GetDistance() << std::endl;
	}
	else if (100 == nCount)
	{
		nCount = 0;
	}
	++nCount;
}

/**
 * Drive left & right motors for 2 seconds then stop
 */
void KBot::AutonomousPeriodic(void)
{
	GetWatchdog().Feed();

	RunRobot(m_pAutonomousController);
}

/**
 * Runs the motors with arcade steering. 
 */
void KBot::TeleopPeriodic(void) 
{
	static int nCount=0;
	
	//std::cerr << "operator control" << std::endl;
	GetWatchdog().Feed();
	
	if (++nCount == 4)
	{
		RunRobot(m_pTeleopController);
		nCount=0;
	}
}

void KBot::RunRobot(Controller* pController)
{
	ReadSensors();			// read all the sensors into robot buffers
	pController->Update();	// update the controller buffers from hardware
	ComputeActuators(pController);		// compute contributions to actuator actions
	UpdateActuators();		// set the motor and actuator states	
}

void KBot::ReadSensors()
{
	m_vecAnalogSensors[GYRO] = m_pGyro->GetAngle();
}

void KBot::ComputeActuators(Controller* pController)
{
	m_vecX[0] = -pController->GetAxis(0);
	m_vecY[0]  = -pController->GetAxis(1);
	float zIn = pController->GetAxis(2);
	float fRotationFactor = 0.02f;
	float fGyroRotation = fRotationFactor*(m_vecAnalogSensors[GYRO]-m_fGyroSetPoint);
	
	if (fabs(zIn) > 0.1f)
	{
		m_fGyroSetPoint = m_vecAnalogSensors[GYRO];
		m_vecR[0] = zIn;
	}
	else
	{
		m_vecR[0] = fGyroRotation;
	}
}

void KBot::UpdateActuators()
{
	UINT8 syncGroup = 0x80;
	
	float fX = std::accumulate(m_vecX.begin(), m_vecX.end(), 0.0f);
	float fY = std::accumulate(m_vecY.begin(), m_vecY.end(), 0.0f);
	float fR = std::accumulate(m_vecR.begin(), m_vecR.end(), 0.0f);
	
	double wheelSpeeds[4];
	wheelSpeeds[0] = fX + fY + fR;
	wheelSpeeds[1] = -fX + fY - fR;
	wheelSpeeds[2] = -fX + fY + fR;
	wheelSpeeds[3] = fX + fY - fR;
	
	// Deadband on wheels:
	for (int i=0; i<4; i++)
	{
		if (fabs(wheelSpeeds[i])<0.05)
		{
			wheelSpeeds[i] = 0.0;
		}
	}

	Normalize(wheelSpeeds);
	m_pLeftJaguarFront->Set(wheelSpeeds[0]*100.0 , syncGroup);
	m_pRightJaguarFront->Set(wheelSpeeds[1]*100.0 , syncGroup);
	m_pRightJaguarBack->Set(wheelSpeeds[2]*100.0, syncGroup);
	m_pLeftJaguarBack->Set(wheelSpeeds[3]*100.0 , syncGroup);

	CANJaguar::UpdateSyncGroup(syncGroup);
}

/**
 * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
 */
void KBot::Normalize(double *wheelSpeeds)
{
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

