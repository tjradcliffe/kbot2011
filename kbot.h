#ifndef KBOT_H
#define KBOT_H

// Framework includes
#include "Gyro.h"
#include "WPILib.h"

// Standard includes (old-style x.h files => cx files)
#include <cmath>

// local forward declarations
class AutonomousController;
class Controller;
class I2C_Ultrasound;
class DistanceSensor;
class TeleopController;

// Sensor identifiers
#define GYRO 0

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
	
	//! Reset the whole robot
	void ResetRobot(bool bRecordTeleop = false);
	
	//! Run the robot based on controller state
	void RunRobot(Controller* pController);
	
	//! Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
	void Normalize(double *pfWheelSpeeds);

	//! Rotate a vecotr (fX, fY) through angle fAngle degrees
	void RotateVector(double &fY, double &fX, double fAngle);

	//! Read all the sensors into robot buffers
	void ReadSensors();
	
	//! Compute actuator inputs
	void ComputeActuators(Controller* pController);
	
	// Compute controller inputs and weight
	void ComputeControllerXYR(Controller* pController);
	
	// Compute gyro inputs and weight
	void ComputeGyroXYR();
	
	//! Actually update the actuators
	void UpdateActuators();

private:
	static const int kPeriodicSpeed;
	
	//! Digital sensors
	int m_nDigitalSensorNumber;
	std::vector<int> m_vecDigitalSensors;
	
	//! Analog sensors
	int m_nAnalogSensorNumber;
	std::vector<float> m_vecAnalogSensors;
	
	//! X-direction update values from various sources
	std::vector<float> m_vecX;
	
	//! Y-direction update values from various sources
	std::vector<float> m_vecY;
	
	//! rotation update values from various sources
	std::vector<float> m_vecR;
	
	//! Weight vector (appliesto x/y/r from each source)
	std::vector<float> m_vecWeightX;
	std::vector<float> m_vecWeightY;
	std::vector<float> m_vecWeightR;
	
	//! Motor controllers
	CANJaguar *m_pLeftJaguarFront;
	CANJaguar *m_pLeftJaguarBack;
	CANJaguar *m_pRightJaguarFront;
	CANJaguar *m_pRightJaguarBack;

	// The gyro is used for maintaining orientation
	Gyro *m_pGyro;
	float m_fGyroSetPoint;
	
	// I2C ultrasound sensor
	I2C_Ultrasound* m_pUltrasound;
	
	// Analog distance sensor
	DistanceSensor* m_pDistanceSensor;
	
	// Declare variables for the controllers
	TeleopController *m_pTeleopController;
	AutonomousController *m_pAutonomousController;

}; // class declaration

#endif // KBOT_H
