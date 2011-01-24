
#include "kbot.h"

// local includes
#include "autonomous_controller.h"
#include "teleop_controller.h"

// standard includes
#include <numeric>

/*!
The constructor builds everything
*/
KBot::KBot(void)
{
	std::cerr << "In Constructor" << std::endl;
	
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
	
	m_pGyro = new Gyro(1);
	
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

	m_pGyro->Reset();	
	
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
	// Runs at 200 Hz
	// feed the user watchdog at every period when disabled
	GetWatchdog().Feed();
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
	//std::cerr << "operator control" << std::endl;
	GetWatchdog().Feed();
	
	RunRobot(m_pTeleopController);
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
	float zIn = -pController->GetAxis(2);
	float fRotationFactor = 0.02f;
	float fGyro = fRotationFactor*m_vecAnalogSensors[GYRO];
	
	if (fabs(zIn) > 0.1f)
	{
		m_pGyro->Reset();
		m_vecR[0] = zIn;
	}
	else
	{
		m_vecR[0] = fGyro;
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

