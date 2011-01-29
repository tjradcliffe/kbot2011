#ifndef KBOT_H
#define KBOT_H

// local includes
#include "mappings.h"

// Framework includes
#include "Gyro.h"
#include "WPILib.h"

// Standard includes (old-style x.h files => cx files)
#include <map>
#include <cmath>

// local forward declarations
class AutonomousController;
class Controller;
class I2C_Ultrasound;
class DistanceSensor;
class TeleopController;

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
	
	//! Apply deadband and normalize all wheel speeds so max is 1.0
	void DeadbandNormalize(double *pfWheelSpeeds);

	//! Rotate a vecotr (fX, fY) through angle fAngle degrees
	void RotateVector(double &fY, double &fX, double fAngle);

	//! Read all the sensors into robot buffers
	void ReadSensors();

	//! Read the ultrasounds (takes some extra logic for cross-talk etc)
	void KBot::ReadUltrasoundSensors();
	
	//! Compute actuator inputs
	void ComputeActuators(Controller* pController);
	
	// Compute controller inputs and weight
	void ComputeControllerXYR(Controller* pController);
	
	// Compute gyro inputs and weight
	void ComputeGyroXYR();
	
	//! Compute the weight given each of the inputs
	void ComputeWeights(Controller* pController);
	
	//! Actually update the actuators
	void UpdateActuators();

	//! Update the wheel speeds
	void UpdateWheelSpeeds();
	
	//! Update the arm and wrist position
	void UpdateArmPosition();
	
	//! Update the roller claw motor speeds
	void UpdateRollerClaw();
	
private:
	static const int kPeriodicSpeed;
	
	//! Digital sensors
	std::map<DigitalMapping, float> m_mapDigitalSensors;
	
	//! Analog sensors
	std::map<AnalogMapping, float> m_mapAnalogSensors;
	
	//! X-direction update values from various sources
	std::map<CalculationMapping, float> m_mapX;
	
	//! Y-direction update values from various sources
	std::map<CalculationMapping, float> m_mapY;
	
	//! rotation update values from various sources
	std::map<CalculationMapping, float> m_mapR;
	
	//! Weight vector (appliesto x/y/r from each source)
	std::map<CalculationMapping, float> m_mapWeightX;
	std::map<CalculationMapping, float> m_mapWeightY;
	std::map<CalculationMapping, float> m_mapWeightR;
	
	//! Motor controllers
	CANJaguar *m_pLeftFrontJaguar;
	CANJaguar *m_pLeftBackJaguar;
	CANJaguar *m_pRightFrontJaguar;
	CANJaguar *m_pRightBackJaguar;

	// The gyro is used for maintaining orientation
	Gyro *m_pGyro;
	float m_fGyroSetPoint;
	
	// I2C ultrasound sensors
	I2C_Ultrasound* m_pLeftUltrasound;
	I2C_Ultrasound* m_pRightUltrasound;
	
	// Analog distance sensors
	DistanceSensor* m_pLeftDistanceSensor;
	DistanceSensor* m_pRightDistanceSensor;
	
	// Declare variables for the controllers
	TeleopController *m_pTeleopController;
	AutonomousController *m_pAutonomousController;

}; // class declaration

#endif // KBOT_H
