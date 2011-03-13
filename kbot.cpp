
#include "kbot.h"

// local includes
#include "autonomous_controller.h"
#include "constants.h"
#include "DistanceSensor.h"
#include "I2C_Ultrasound.h"
#include "score_three_controller.h"
#include "teleop_controller.h"
#include "KbotPID.h"

// FRC includes

//#define USE_I2C
#ifdef USE_I2C
#include "I2C.h"
#endif

// standard includes
#include <numeric>

const int KBot::kPeriodicSpeed = 50; // Speed of periodic loops in Hz

// PID parameters for asymmetric PID:
const float KBot::k_posP =0.015;  // pos values = DOWN on our robot
const float KBot::k_posI =0.04;
const float KBot::k_posD =0.025;
const float KBot::k_negP =0.25;//0.1;	// neg values = UP on our robot
const float KBot::k_negI =0.1;
const float KBot::k_negD =0.5;

const float KBot::kArmGain = 5.0;

// One and two-tube scoring filenames
std::string strScoreOne = "score_one.dat";
std::string strScoreTwo = "score_two.dat";

// Jag mode (CANJaguar::kSpeed/CANJaguar::kVoltage) and constant (150/12)
const CANJaguar::ControlMode knDriveJaguarMode = CANJaguar::kSpeed;
const float kfDriveJaguarConstant = 150;

static int autoCount=0;

/*!
The constructor builds everything
*/
KBot::KBot(void)
{
	std::cerr << "In Constructor" << std::endl;
	IterativeRobot::SetPeriod(1.0/kPeriodicSpeed);
	std::cerr << "Periodic rate ="<< IterativeRobot::GetLoopsPerSec() << " loops per second." << std::endl;
	
	// Build the Jags
	BuildJags();
	
#ifdef PID_ARM
	// Arm PID controller:
	m_pArmPID = new KbotPID(k_posP, 0.01, 0.0);
#endif

	// do not over-ride record button by default
	m_bRecordOverride = false;
	m_bPreviousOverride = false;
	
	// Line following PID
	m_pLinePID = new KbotPID(1.0, 0.0, 2.0);
	m_nLineCount = 0;
	
	// for stopping on line
	m_nStoppedCount = -1;
	
	// Arm angle potentiometer
	m_pArmAngle = new AnalogChannel(knAnalogSlot, knArmAngle);

	m_nDeployerPosition = 0;	// IN
	m_nWristPosition = 0;		// FOLDED
	m_nJawPosition = 0;			// CLOSED
	
	// Solenoids to control wrist, jaw and swing-arm
	m_pWristOutSolenoid = new Solenoid(knRelaySlot, knWristOutSolenoid);
	m_pWristInSolenoid = new Solenoid(knRelaySlot, knWristInSolenoid);
	m_pJawOpenSolenoid = new Solenoid(knRelaySlot, knJawOpenSolenoid);
	m_pJawClosedSolenoid = new Solenoid(knRelaySlot, knJawClosedSolenoid);
	m_pDeployerOutSolenoid = new Solenoid(knRelaySlot, knDeployerOutSolenoid);
	m_pDeployerInSolenoid = new Solenoid(knRelaySlot, knDeployerInSolenoid);
	
	// Light relays
	m_pBlueLightRelay = new Relay(knBlueLightRelay, Relay::kForwardOnly);
	m_pRedLightRelay = new Relay(knRedLightRelay, Relay::kForwardOnly);
	m_pWhiteLightRelay = new Relay(knWhiteLightRelay, Relay::kForwardOnly);
	m_nLightState = knAllLightsOff;

	// Compressor controls
	m_pCompressorRelay = new Relay(knCompressorRelay,Relay::kForwardOnly);
	m_pCompressorLimit = new DigitalInput(knCompressorLimit);
	
	// controllers
	m_pTeleopController = new TeleopController();
	m_pPlaybackController = new AutonomousController(this);
	
	// gyro
	m_pGyro = new Gyro(knAnalogSlot, knGyro);
	m_fGyroSetPoint = 0.0f;

	// accelerometer
	//m_pAccelerometer = new ADXL345_I2C(knDigitalSlot, ADXL345_I2C::kRange_2G);
	
	// ultrasounds
#ifdef USE_I2C
	m_pLeftUltrasound = new I2C_Ultrasound(knLeftUltrasound);
	m_pRightUltrasound = new I2C_Ultrasound(knRightUltrasound);
#endif
	
	// analog distance sensors
	m_pLeftIRSensor = new DistanceSensor(knLeftIRSensor);
	m_pRightIRSensor = new DistanceSensor(knRightIRSensor);
	
	// line sensors
	m_pLineRight = new DigitalInput(knDigitalSlot, knLineRight);
	m_pLineLeft = new DigitalInput(knDigitalSlot, knLineLeft);
	m_pLineBack = new DigitalInput(knDigitalSlot, knLineBack);
	
	// tube sensors
	m_pTubeLeft = new DigitalInput(knDigitalSlot, knTubeLeft);
	m_pTubeRight = new DigitalInput(knDigitalSlot, knTubeRight);
	m_pTubeIR = new AnalogChannel(knAnalogSlot, knTubeIR);
	
	// retro-reflector (if we use it)
	m_pRetroReflector = new DigitalInput(knDigitalSlot, knRetroReflector);
	
	// switch inputs
	m_pRecordSwitch = new DigitalInput(knDigitalSlot, knRecordSwitch);
	m_pRecoverSwitch = new DigitalInput(knDigitalSlot, knRecoverSwitch);
	m_pMirrorSwitch = new DigitalInput(knDigitalSlot, knMirrorSwitch);
	m_pOneTwoTubeSwitch = new DigitalInput(knDigitalSlot, knOneTwoTubeSwitch);
	m_pFifthSwitch = new DigitalInput(knDigitalSlot, knFifthSwitch);
}

/*!
Function for (re)building Jags
*/
void KBot::BuildJags()
{
	// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
	m_pRightFrontJaguar = new CANJaguar(knRightFrontJaguar, knDriveJaguarMode);
	m_pRightFrontJaguar->Set(0.0f);
	m_pLeftFrontJaguar = new CANJaguar(knLeftFrontJaguar, knDriveJaguarMode);
	m_pLeftFrontJaguar->Set(0.0f);
	m_pRightBackJaguar = new CANJaguar(knRightBackJaguar, knDriveJaguarMode);
	m_pRightBackJaguar->Set(0.0f);
	m_pLeftBackJaguar = new CANJaguar(knLeftBackJaguar, knDriveJaguarMode);
	m_pLeftBackJaguar->Set(0.0f);
	
	// Time in seconds for Jag to reset after a fault.  0.5 is as low as possible.
	m_pRightFrontJaguar->ConfigFaultTime(0.5f);
	m_pLeftFrontJaguar->ConfigFaultTime(0.5f);
	m_pRightBackJaguar->ConfigFaultTime(0.5f);
	m_pLeftBackJaguar->ConfigFaultTime(0.5f);
	
	m_vecJags.clear();	// throw away any old data
	m_vecJagErrors.clear();
	m_vecJags.push_back(m_pRightFrontJaguar);
	m_vecJags.push_back(m_pLeftFrontJaguar);
	m_vecJags.push_back(m_pRightBackJaguar);
	m_vecJags.push_back(m_pLeftBackJaguar);
	m_vecJagErrors.push_back(0);
	m_vecJagErrors.push_back(0);
	m_vecJagErrors.push_back(0);
	m_vecJagErrors.push_back(0);
	
	// Arm actuators
	m_pArmJaguar = new CANJaguar(knArmJaguar, CANJaguar::kVoltage);
	m_pLowerRollerJaguar = new CANJaguar(knLowerRollerJaguar, CANJaguar::kVoltage);
	m_pUpperRollerJaguar = new CANJaguar(knUpperRollerJaguar, CANJaguar::kVoltage);	
}

void KBot::ControlCompressor(void)
{
	// control the compressor based on pressure switch reading
	if (0 == m_mapDigitalSensors[knCompressorLimit])
	{
		m_pCompressorRelay->SetDirection(Relay::kForwardOnly);
		m_pCompressorRelay->Set(Relay::kForward);
	}
	else
	{
		m_pCompressorRelay->SetDirection(Relay::kForwardOnly);
		m_pCompressorRelay->Set(Relay::kOff);				
	}
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
	
	m_pCompressorRelay->SetDirection(Relay::kForwardOnly);

	m_nLineCount = 0;
	
	m_pBlueLightRelay->Set(Relay::kOff);
	m_pRedLightRelay->Set(Relay::kOff);
	m_pWhiteLightRelay->Set(Relay::kOff);

	// handle switches
	if (0 == m_pRecoverSwitch->Get())
	{
		// TODO: copy files from backups
	}
	
	if ((0 == m_pRecordSwitch->Get()) || m_bRecordOverride)	// we are recording
	{
		if (0 == m_pOneTwoTubeSwitch->Get())	
		{
			m_pTeleopController->SetFilename(strScoreOne);
		}
		else
		{
			m_pTeleopController->SetFilename(strScoreTwo);			
		}
		
		if (bRecordTeleop)	// this tells use if we are opening/closing file
		{
			m_pTeleopController->Reset();
		}
		else
		{
			m_pTeleopController->Done();
			m_bRecordOverride = false;	// no longer recording
		}
	}
	else // we are not recording
	{
		m_pTeleopController->SetFilename("");
		m_pTeleopController->Reset();		
		m_pPlaybackController->Reset();
		
		if (0 == m_pOneTwoTubeSwitch->Get())	
		{
			m_pPlaybackController->SetFilename(strScoreOne);
		}
		else
		{
			m_pPlaybackController->SetFilename(strScoreTwo);			
		}
		
		if (0 == m_pMirrorSwitch->Get())	
		{
			m_pPlaybackController->SetMirror(true);
		}
		else
		{
			m_pPlaybackController->SetMirror(false);			
		}
	}
}

/*!
Initialize the jags
 */
void KBot::InitJags()
{
	m_pLeftFrontJaguar->SetSafetyEnabled(true);
	m_pLeftBackJaguar->SetSafetyEnabled(true);
	m_pRightFrontJaguar->SetSafetyEnabled(true);
	m_pRightBackJaguar->SetSafetyEnabled(true);
	m_pArmJaguar->SetSafetyEnabled(true);
	m_pLowerRollerJaguar->SetSafetyEnabled(true);
	m_pUpperRollerJaguar->SetSafetyEnabled(true);

	m_pLeftFrontJaguar->SetPID(1.0, 0.0, 0.0);
	m_pLeftBackJaguar->SetPID(1.0, 0.0, 0.0);
	m_pRightFrontJaguar->SetPID(1.0, 0.0, 0.0);
	m_pRightBackJaguar->SetPID(1.0, 0.0, 0.0);

	m_pLowerRollerJaguar->SetPID(1.0, 0.0, 0.0);
	m_pUpperRollerJaguar->SetPID(1.0, 0.0, 0.0);
	
	if (knDriveJaguarMode == CANJaguar::kSpeed)
	{
		m_pLeftFrontJaguar->ConfigEncoderCodesPerRev(360);
		m_pLeftBackJaguar->ConfigEncoderCodesPerRev(360);
		m_pRightFrontJaguar->ConfigEncoderCodesPerRev(360);
		m_pRightBackJaguar->ConfigEncoderCodesPerRev(360);
	}	
	m_pLeftFrontJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pLeftBackJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pRightFrontJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pRightBackJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pArmJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
	m_pLowerRollerJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_pUpperRollerJaguar->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	
	m_pArmJaguar->EnableControl();
	m_pLowerRollerJaguar->EnableControl();
	m_pUpperRollerJaguar->EnableControl();	
	m_pLeftFrontJaguar->EnableControl();//error?
	m_pLeftBackJaguar->EnableControl();
	m_pRightFrontJaguar->EnableControl();
	m_pRightBackJaguar->EnableControl();
}

/*!
This is the initialization code that gets called when the robot
is created. 
*/
void KBot::RobotInit()
{
	GetWatchdog().SetEnabled(true);

	AnalogModule::GetInstance(knAnalogSlot)->SetSampleRate(50000); //50 * (1<<5));//sampleRate);

	InitJags();
	
#ifdef PID_ARM
	// Arm PID controller:
	// First 3 parameters are positive dir (down on our robot), next 3 are negative (up)
	m_pArmPID->setAsymmetricPID(k_posP, k_posI, k_posD,  k_negP, k_negI, k_negD);
	m_pArmPID->setDesiredValue(845.0);
	m_pArmPID->setErrorEpsilon(20.0);  // was 10.0 and working quite well
	m_pArmPID->setMaxOutput(1.0); // Max range
	m_pArmPID->setMinOutput(-1.0); // Max range
	m_pArmPID->setDeadBand(0.4); // about +=2.4V
#endif
	
	m_pLinePID->setDesiredValue(0.0);
	m_pLinePID->setMaxOutput(1.0); // Max range
	m_pLinePID->setMinOutput(-1.0); // Max range
	
	m_pCompressorRelay->SetDirection(Relay::kForwardOnly);
	
	m_pGyro->Reset();
	m_pGyro->SetSensitivity(0.007); // 7 mV/deg/s
	m_mapAnalogSensors[knGyro] = m_pGyro->GetAngle();

#ifdef USE_I2C
	m_pLeftUltrasound->SetRange(I2C_Ultrasound::kMaxRange);
	m_pLeftUltrasound->SetMaxGain(I2C_Ultrasound::kSetGain350);
	m_pRightUltrasound->SetRange(I2C_Ultrasound::kMaxRange);
	m_pRightUltrasound->SetMaxGain(I2C_Ultrasound::kSetGain350);
	
	// Reprogram Ultrasound's I2C address:
	//   Create the Ultrasound with an address of E0 (for an unprogrammed Ultrasound)
	//   then call this method to reprogram it.
	//m_pUltrasound->SetI2CAddress(0xe2);
#endif
	
	m_pLeftIRSensor->SetBestFitParameters(DistanceSensor::kAIRRSv2Exponent, DistanceSensor::kAIRRSv2Multiplier);
	m_pRightIRSensor->SetBestFitParameters(DistanceSensor::kAIRRSv2Exponent, DistanceSensor::kAIRRSv2Multiplier);
	
	m_fTargetArmAngle = m_pArmAngle->GetValue();  // whatever current position is
	m_fArmSpeed = 0.0f;
	m_nWristPosition = 0;  // in
	m_nJawPosition = 0;  // open
	m_nDeployerPosition = 0; // in
	
	m_fLowerJawRollerSpeed = 0.0f;
	m_fUpperJawRollerSpeed = 0.0f;
}

/*!
Called at start of disabled
*/
void KBot::DisabledInit() 
{
	// do NOT reset robot here!
#ifdef PID_ARM
	delete m_pArmPID; // Force save of data file
	m_pArmPID = 0;
#endif
	
	GetWatchdog().SetEnabled(false);
	m_pLeftFrontJaguar->SetSafetyEnabled(false);
	m_pLeftBackJaguar->SetSafetyEnabled(false);
	m_pRightFrontJaguar->SetSafetyEnabled(false);
	m_pRightBackJaguar->SetSafetyEnabled(false);
	m_pArmJaguar->SetSafetyEnabled(false);
	m_pLowerRollerJaguar->SetSafetyEnabled(false);
	m_pUpperRollerJaguar->SetSafetyEnabled(false);
}

/*!
Called at start of autonomous
*/
void KBot::AutonomousInit() 
{
#ifdef PID_ARM
	delete m_pArmPID; // Force save of data file and re-create
	m_pArmPID = new KbotPID(0.0, 0.0, 0.0);
	m_pArmPID->setAsymmetricPID(k_posP, k_posI, k_posD,  k_negP, k_negI, k_negD);
#endif
	
	m_pLeftFrontJaguar->SetSafetyEnabled(true);
	m_pLeftBackJaguar->SetSafetyEnabled(true);
	m_pRightFrontJaguar->SetSafetyEnabled(true);
	m_pRightBackJaguar->SetSafetyEnabled(true);
	m_pArmJaguar->SetSafetyEnabled(true);
	m_pLowerRollerJaguar->SetSafetyEnabled(true);
	m_pUpperRollerJaguar->SetSafetyEnabled(true);

	ResetRobot();
	
	//Check switches and set autonomous mode
	//m_autoMode = 1; //0=playback mode, 1=programmed sequence
	if (0 == m_pOneTwoTubeSwitch->Get())
	{
		m_autoMode = 0;
	} else
	{
		m_autoMode = 1;
	}
	autoCount = 0;
}

/*!
Note that for teleop we may want to preserve some information
here, so a full reset may not be what we need.
*/
void KBot::TeleopInit() 
{
#ifdef PID_ARM
	delete m_pArmPID; // Force save of data file and re-create
	m_pArmPID = new KbotPID(0.0, 0.0, 0.0);
	m_pArmPID->setAsymmetricPID(k_posP, k_posI, k_posD,  k_negP, k_negI, k_negD);
#endif
	
	m_pLeftFrontJaguar->SetSafetyEnabled(true);
	m_pLeftBackJaguar->SetSafetyEnabled(true);
	m_pRightFrontJaguar->SetSafetyEnabled(true);
	m_pRightBackJaguar->SetSafetyEnabled(true);
	m_pArmJaguar->SetSafetyEnabled(true);
	m_pLowerRollerJaguar->SetSafetyEnabled(true);
	m_pUpperRollerJaguar->SetSafetyEnabled(true);
	
	ResetRobot(true);  // true so we get new file for recording if we are recording
}

/********************************** Periodic Routines *************************************/

void KBot::DisabledPeriodic(void)  
{
	static int nCount = 0;
	
	// feed the user watchdog at every period when disabled
	GetWatchdog().Feed();

	ReadSensors();	// fill sensor arrays

	m_pTeleopController->Update();
	if (m_pTeleopController->GetButton(knRecordOverride) && !m_bPreviousOverride)
	{
		m_bPreviousOverride = true;
		m_bRecordOverride = !m_bRecordOverride;
	}
	else
	{
		m_bPreviousOverride = false;		
	}
	//UpdateDriverStation();	// will show line sensors as well
	
	if (nCount == 100)  // once per two seconds
	{
		nCount = 0;
	
//#define CONTROLLER_DEBUG
//#define ANALOG_DEBUG
//#define DIGITAL_DEBUG
#ifdef CONTROLLER_DEBUG
		m_pTeleopController->Update();
		const std::vector<int>& vecButtons = m_pTeleopController->GetButtons();
		for(unsigned int nIndex = 0; nIndex < vecButtons.size()/2; ++nIndex)
		{
			if (vecButtons[nIndex] == 1)
			{
				std::cerr << "(" << nIndex << " : " << vecButtons[nIndex] << ") ";
			}
		}
		std::cerr << std::endl;
		for(unsigned int nIndex = vecButtons.size()/2; nIndex < vecButtons.size(); ++nIndex)
		{
			if (vecButtons[nIndex] == 1)
			{
				std::cerr << "(" << nIndex << " : " << vecButtons[nIndex] << ") ";
			}
		}
		std::cerr << std::endl;
		const std::vector<float>& vecAxes = m_pTeleopController->GetAxes();
		for(unsigned int nIndex = 0; nIndex < vecAxes.size(); ++nIndex)
		{
			std::cerr << "(" << nIndex << " : " << vecAxes[nIndex] << ") ";
		}
		std::cerr << std::endl << "===========" << std::endl;
#endif
		
#ifdef ANALOG_DEBUG
		// display analog inputs
		std::map<AnalogMapping, float>::iterator itAnalog = m_mapAnalogSensors.begin();
		std::cerr.precision(4);
		for(; itAnalog != m_mapAnalogSensors.end(); ++itAnalog)
		{
			std::cerr << itAnalog->first << ": " << itAnalog->second << " | ";
		}
		std::cerr << std::endl;
#endif
		
#ifdef DIGITAL_DEBUG
		// display digital inputs
		std::map<DigitalMapping, int>::iterator itDigital = m_mapDigitalSensors.begin();
		for(; itDigital != m_mapDigitalSensors.end(); ++itDigital)
		{
			std::cerr << itDigital->first << ": " << itDigital->second << " | ";
		}
		std::cerr << std::endl;
		std::cerr << "===========" << std::endl;
#endif
	}
	++nCount;
}

/**
Called on a clock during autonomous
 */
void KBot::AutonomousPeriodic(void)
{
	double wheelSpeeds[4];  // signs changed to get rotation right

	
	GetWatchdog().Feed();
	if (m_autoMode==0) 
	{
		RunRobot(m_pPlaybackController);
	} else if (m_autoMode==1)
	{
		ReadSensors();			// read all the sensors into robot buffers
		m_pPlaybackController->Update();	// update the controller buffers from hardware
		if (autoCount==0)
		{
			wheelSpeeds[0]=0.0;
			wheelSpeeds[1]=0.0;
			wheelSpeeds[2]=0.0;
			wheelSpeeds[3]=0.0;
			m_fLowerJawRollerSpeed=0;
			m_fUpperJawRollerSpeed=0;
			m_fArmSpeed=0;
			m_nJawPosition = 0;
			m_nWristPosition = 0;
			
		} else if (autoCount<50*0.5) // Feed in tube for 0.5 second
		{
			wheelSpeeds[0]=0.0;
			wheelSpeeds[1]=0.0;
			wheelSpeeds[2]=0.0;
			wheelSpeeds[3]=0.0;
			m_fLowerJawRollerSpeed=0.75;
			m_fUpperJawRollerSpeed=-0.75;
			
		} else if (autoCount<50*6.2) // Drive forward 5.7 secs at 3/4 speed
		{							// while lifting arm
			wheelSpeeds[0]=0.75;
			wheelSpeeds[1]=0.75;
			wheelSpeeds[2]=0.75;
			wheelSpeeds[3]=0.75;
			m_fArmSpeed = -3.0;
			m_fLowerJawRollerSpeed=0.0;
			m_fUpperJawRollerSpeed=0.0;
			
		} else if (autoCount<50*6.7) // Rotate tube forward for 0.5 sec
		{
			wheelSpeeds[0]=0.0;
			wheelSpeeds[1]=0.0;
			wheelSpeeds[2]=0.0;
			wheelSpeeds[3]=0.0;
			m_fArmSpeed = 0.0;
			m_fLowerJawRollerSpeed=-0.75;
			m_fUpperJawRollerSpeed=-0.75;
		
		} else if (autoCount<50*7.2) // Open jaw (0.5 sec)
		{
			wheelSpeeds[0]=0.0;
			wheelSpeeds[1]=0.0;
			wheelSpeeds[2]=0.0;
			wheelSpeeds[3]=0.0;
			m_fArmSpeed = 0.0;
			m_fLowerJawRollerSpeed=0.0;
			m_fUpperJawRollerSpeed=0.0;
			m_nJawPosition = 1;
		
		} else if (autoCount<50*8.0) // Turn away (0.8 sec)
		{
			wheelSpeeds[0]=0.5;
			wheelSpeeds[1]=-0.5;
			wheelSpeeds[2]=-0.5;
			wheelSpeeds[3]=0.5;
			m_fArmSpeed = 0.0;
			m_fLowerJawRollerSpeed=0.0;
			m_fUpperJawRollerSpeed=0.0;
		
		} else  // Stop everything!!
		{
			wheelSpeeds[0]=0.0;
			wheelSpeeds[1]=0.0;
			wheelSpeeds[2]=0.0;
			wheelSpeeds[3]=0.0;
			m_fArmSpeed = 0.0;
			m_fLowerJawRollerSpeed=0.0;
			m_fUpperJawRollerSpeed=0.0;
		
		}
		autoCount++;
		m_pLeftFrontJaguar->Set(wheelSpeeds[0]*kfDriveJaguarConstant);// , syncGroup);
		Wait(0.001);
		m_pRightFrontJaguar->Set(-wheelSpeeds[1]*kfDriveJaguarConstant);// , syncGroup);
		Wait(0.001);
		m_pRightBackJaguar->Set(-wheelSpeeds[2]*kfDriveJaguarConstant);//, syncGroup);
		Wait(0.001);
		m_pLeftBackJaguar->Set(wheelSpeeds[3]*kfDriveJaguarConstant);// , syncGroup);
		Wait(0.001);
		m_pLowerRollerJaguar->Set(10*m_fLowerJawRollerSpeed);// , syncGroup);
		Wait(0.001);
		m_pUpperRollerJaguar->Set(10*m_fUpperJawRollerSpeed);// , syncGroup);
		Wait(0.001);
		m_pArmJaguar->Set(m_fArmSpeed);//, syncGroup);
		UpdateActuators();		// set the motor and actuator states
		ControlCompressor();	// manage the compressor state based on switch state
		//UpdateDriverStation();	// update the driver station 
	}
}

/**
 Called on a clock during teleop
 */
void KBot::TeleopPeriodic(void) 
{
	//std::cerr << "operator control" << std::endl;
	GetWatchdog().Feed();
	
	RunRobot(m_pTeleopController);

	/*bool bNewErrors = false;
	for(unsigned int nIndex = 0; nIndex < m_vecJags.size(); ++nIndex)
	{
		int nError = m_vecJags[nIndex]->GetFaults();
		
		if (nError != m_vecJagErrors[nIndex])
		{
			std::cerr << nIndex+1 << " " << nError << " | ";
			m_vecJagErrors[nIndex] = nError;
			bNewErrors = true;
		}
	}
	if (bNewErrors)	// newline if we have seen new errorss
	{
		std::cerr << std::endl;
	}*/
	
	static int nCount = 0;	
	if (nCount == 50)  // once per second
	{
		/*if (0 != m_pLeftFrontJaguar->GetFaults() + m_pRightFrontJaguar->GetFaults() + m_pLeftBackJaguar->GetFaults() + m_pRightBackJaguar->GetFaults() + m_pArmJaguar->GetFaults() + m_pUpperRollerJaguar->GetFaults() + m_pLowerRollerJaguar->GetFaults() )
		{
			std::cerr << m_pLeftFrontJaguar->GetFaults() << " " << m_pRightFrontJaguar->GetFaults() << " " << m_pLeftBackJaguar->GetFaults() << " " << m_pRightBackJaguar->GetFaults() << " " << m_pArmJaguar->GetFaults() << " " << m_pUpperRollerJaguar->GetFaults() << " " << m_pLowerRollerJaguar->GetFaults() << " " <<std::endl; 
		}*/
		//std::cerr << m_pLeftFrontJaguar->GetBusVoltage() << " " << m_pRightFrontJaguar->GetBusVoltage() << " " << m_pLeftBackJaguar->GetBusVoltage() << " " << m_pRightBackJaguar->GetBusVoltage() << " " << m_pArmJaguar->GetBusVoltage() << " " << m_pUpperRollerJaguar->GetBusVoltage() << " " << m_pLowerRollerJaguar->GetBusVoltage() << " " <<std::endl; 

		nCount = 0;
		//std::cerr << m_pLeftFrontJaguar->GetTemperature() << " " << m_pRightFrontJaguar->GetTemperature() << " " << m_pLeftBackJaguar->GetTemperature() << " " << m_pRightBackJaguar->GetTemperature() << " " << m_pArmJaguar->GetTemperature() << " " << m_pUpperRollerJaguar->GetTemperature() << " " << m_pLowerRollerJaguar->GetTemperature() << " " <<std::endl; 
		//std::cerr << wheelSpeeds[0] << "  " << wheelSpeeds[1] << "  " << "  " << wheelSpeeds[2] << "  " << wheelSpeeds[3] <<std::endl;
		//std::cerr << m_pLeftFrontJaguar->GetOutputVoltage() << "  " << m_pRightFrontJaguar->GetOutputVoltage() << "  " <<m_pLeftBackJaguar->GetOutputVoltage() << "  " <<m_pRightBackJaguar->GetOutputVoltage() << "  " <<std::endl;
	}
	++nCount;
	
}

/*!
Update extended IO module of driver-station.  Currently does nothing.
*/
void KBot::UpdateDriverStation()
{
	try
	{
		DriverStationEnhancedIO& dseio = DriverStation::GetInstance()->GetEnhancedIO();
		dseio.GetDigitalConfig(1);

		unsigned char nBitMask = 0;
		if (0 == m_mapDigitalSensors[knLineRight])
		{
			nBitMask += 1 << 7;
		}
		if (0 == m_mapDigitalSensors[knLineLeft])
		{
			nBitMask += 1;
		}
		
		if (m_bRecordOverride)	// make a pair of lights to be obvious
		{
			nBitMask += 1 << 3;
			nBitMask += 1 << 4;
		}
		
		dseio.SetLEDs(nBitMask);  // led bitmask
	}
	catch(...)
	{
		// do nothing... just in case we throw when not connected etc
	}
}

void KBot::RunRobot(Controller* pController)
{
	ReadSensors();			// read all the sensors into robot buffers
	pController->Update();	// update the controller buffers from hardware
	ComputeActuators(pController);		// compute contributions to actuator actions
	ComputeWeights(pController);	// compute the weights for each input
	UpdateActuators();		// set the motor and actuator states
	ControlCompressor();	// manage the compressor state based on switch state
	//UpdateDriverStation();	// update the driver station 
}

/*!
Read or ping the ultrasounds, depending on where we are
in an internal loop.  They are pinged at different times,
which may create issues with the control logic.
*/
#ifdef USE_I2C
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
		m_pRightUltrasound->Ping();
	}
	else if (10 == nUltrasoundCount)
	{
		nUltrasoundCount = 0;
		float fDistance = m_pRightUltrasound->GetDistance();
		if (10.0 > fDistance)
		{
			m_mapAnalogSensors[knRightUltrasound] = fDistance;
		}
	}
	++nUltrasoundCount;
}
#endif

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
	
	// NOTE Right IR sensor is set 18 cm back from left, which is on the front
	// of the robot
	m_mapAnalogSensors[knLeftIRSensor] = m_pLeftIRSensor->GetDistance();
	m_mapAnalogSensors[knRightIRSensor] = m_pRightIRSensor->GetDistance()-12.0f;
	
#ifdef USE_I2C
	ReadUltrasoundSensors();	// put ping logic in its own method
#endif
	
	m_mapAnalogSensors[knArmAngle] = m_pArmAngle->GetValue();
	m_mapAnalogSensors[knTubeIR] = m_pTubeIR->GetValue();
	
#ifdef USE_I2C
	ADXL345_I2C::AllAxes accelerations = m_pAccelerometer->GetAccelerations();
	m_mapAnalogSensors[knAccelerationX] = accelerations.XAxis;
	m_mapAnalogSensors[knAccelerationY] = accelerations.YAxis;
	m_mapAnalogSensors[knAccelerationZ] = accelerations.ZAxis;
#endif
	
	//*********DIGITAL SENSORS************
	
	m_mapDigitalSensors[knLineRight] = m_pLineRight->Get();
	m_mapDigitalSensors[knLineLeft] = m_pLineLeft->Get();
	m_mapDigitalSensors[knLineBack] = m_pLineBack->Get();
	m_mapDigitalSensors[knRetroReflector] = m_pRetroReflector->Get();
	m_mapDigitalSensors[knTubeLeft] = m_pTubeLeft->Get();
	m_mapDigitalSensors[knTubeRight] = m_pTubeRight->Get();
	m_mapDigitalSensors[knCompressorLimit] = m_pCompressorLimit->Get();
	m_mapDigitalSensors[knRecordSwitch] = m_pRecordSwitch->Get();
	m_mapDigitalSensors[knOneTwoTubeSwitch] = m_pOneTwoTubeSwitch->Get();
	m_mapDigitalSensors[knMirrorSwitch] = m_pMirrorSwitch->Get();
	m_mapDigitalSensors[knRecoverSwitch] = m_pRecoverSwitch->Get();
	m_mapDigitalSensors[knFifthSwitch] = m_pFifthSwitch->Get();	
}

/*!
Light logic is as follows:

\param pController controller running the robot

Red/light/blue on for appropriate buttons
All lights for "all lights" button
Top and Bottom lights flash when tube in place in roller claw
Otherwise all lights off
*/
void KBot::ComputeLights(Controller* pController)
{
	if (pController->GetButton(knRedTubeButton))
	{
		m_nLightState = knRedLight;
	}
	else if (pController->GetButton(knBlueTubeButton))
	{
		m_nLightState = knBlueLight;
	}
	else if (pController->GetButton(knWhiteTubeButton))
	{
		m_nLightState = knWhiteLight;
	}
	else if (pController->GetButton(knAllLightsButton))
	{
		m_nLightState = knAllLightsOn;
	}
	else if ((m_mapDigitalSensors[knTubeRight] == 0) && (m_mapDigitalSensors[knTubeLeft] == 0))
	{
		if (m_nWristPosition == 1)
		{
			m_nLightState = knTubeCaptureSignal;
		}
		else
		{
			m_nLightState = knAllLightsOff;		
		}
	}	
	else
	{
		m_nLightState = knAllLightsOff;
	}
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
}

/*!
Compute the rotation we want based on gyroscopic station-keeping
*/
void KBot::ComputeGyroXYR()
{
	m_mapX[knGyroTracking] = 0;	
	m_mapY[knGyroTracking] = 0;	
	m_mapR[knGyroTracking] = kfRotationFactor*(m_mapAnalogSensors[knGyro]-m_fGyroSetPoint);	
}

/*!
Compute the rotation we want based on line following
*/
void KBot::ComputeLineAndWallXYR()
{
	static float fLineFollowingStartSpeed = 1.0f;	// speed we start at
	static float fLineFollowingMinSpeed = 0.35f;	// speed we ramp down to
	static int nMaxCount = 50;		// counts we ramp down over (50 == 1 second)
	static int nMaxStopped = 25;		// counts we reverse for (25 == 0.5 second)
	static float fReverseSpeed = 0.85f;	// speed we reverse in
	static int nRotation = 50;  // rotation correction (very forgiving 10 - 100??)
	
	float fLineFollowingSpeed = fLineFollowingStartSpeed*(nMaxCount-m_nLineCount)/nMaxCount;
	fLineFollowingSpeed = fLineFollowingSpeed>fLineFollowingMinSpeed?fLineFollowingSpeed:fLineFollowingMinSpeed;
	++m_nLineCount;
	
	m_mapX[knLineFollowing] = 0;	
	m_mapY[knLineFollowing] = fLineFollowingSpeed;	
	float fSignal = 0;
	if (0 == m_mapDigitalSensors[knLineRight])
	{
		fSignal += 1;
	}
	if (0 == m_mapDigitalSensors[knLineLeft])
	{
		fSignal -= 1;		
	}
	m_mapR[knLineFollowing] = -nRotation*m_mapY[knLineFollowing]*kfRotationFactor*fSignal;

	if ((0 == m_mapDigitalSensors[knLineRight]) && (0 == m_mapDigitalSensors[knLineLeft]) && (m_nStoppedCount < nMaxStopped))
	{
		++m_nStoppedCount;
		m_mapY[knLineFollowing] = -fReverseSpeed; // little bit backward
	}
	else if ((m_nStoppedCount > 0) && (m_nStoppedCount < nMaxStopped))
	{
		++m_nStoppedCount;
		m_mapY[knLineFollowing] = -fReverseSpeed; // little bit backward		
	}
	else if (m_nStoppedCount >= nMaxStopped)
	{
		m_mapY[knLineFollowing] = 0.0f; // STOP			
	}

#ifdef NOT_NOW
	float fRightWallDistance = m_mapAnalogSensors[knRightIRSensor];
	float fLeftWallDistance = m_mapAnalogSensors[knLeftIRSensor];
	float fLowerDistance = (fRightWallDistance<fLeftWallDistance)?fRightWallDistance:fLeftWallDistance;
	float fMinDistance = 50.0f;
	float fMaxDistance = 250.0f;
	m_mapX[knWallAlign] = 0;	
	m_mapY[knWallAlign] = 0;	
	m_mapR[knWallAlign] = 0;
	if (fLowerDistance < fMaxDistance)
	{
		// effectively subract off some portion of the approach velocity
		m_mapY[knWallAlign] = -0.75*m_mapY[knLineFollowing]*(fMaxDistance-fLowerDistance)/(fMaxDistance-fMinDistance);
	}
	else if (fLowerDistance < fMinDistance)
	{
		m_mapY[knLineFollowing] = 0;
		m_mapY[knWallAlign] = 0;
	}
#endif
	
}

/*!
Compute what we want the robot to do in terms of XYR for the
main body and angle, wrist and roller state for the arm.
*/
void KBot::ComputeActuators(Controller* pController)
{
	ComputeControllerXYR(pController);
	ComputeGyroXYR();
	ComputeLineAndWallXYR();
	ComputeArmAndDeployer(pController);
	ComputeLights(pController);
}

/*!
Compute all the arm functions from the controller values
*/
void KBot::ComputeArmAndDeployer(Controller *pController)
{
	static int nAutoScoreCount = 0;	// local static for autoscore
	if (pController->GetAxis(knAutoScoreAxis) < -0.8f)
	{
		if (nAutoScoreCount < 10) // 0.2 s
		{
			// starting with roll-out
			pController->SetAxis(knRollInOut, 1.0f);
		}
		else if (nAutoScoreCount < 30)
		{
			pController->SetButton(knJawOpen, true);
			pController->SetAxis(knArmUpDown, 1.0f);			
		}
		else if (nAutoScoreCount < 50)
		{
			pController->SetAxis(knArmUpDown, 1.0f);						
		}
		++nAutoScoreCount;
	}
	else
	{
		nAutoScoreCount = 0; // reset the counter for next go
	}
	
	if (pController->GetButton(knDeployerOutButton))
	{
		// TODO: NEED TO PUT ARM UP!!!
		m_nDeployerPosition = 1;
	}
	else
	{
		// TODO: NEED TO PUT ARM UP!!!
		m_nDeployerPosition = 0;
	}
		
	if (pController->GetButton(knArmParked))
	{
		m_fTargetArmAngle = 845.0f;
		m_nWristPosition = 0;
		m_fLowerJawRollerSpeed = 0.0;
		m_fUpperJawRollerSpeed = 0.0;
	}
	else
	{
		if (pController->GetButton(knWristIn))
		{
			m_nWristPosition = 0;
		}
		else if (pController->GetButton(knWristOut))
		{
			m_nWristPosition = 1;
		}
		if (pController->GetButton(knJawOpen))
		{
			m_nJawPosition = 1;
		}
		else
		{
			m_nJawPosition = 0;
		}
		
		// respond to pre-set arm position buttons
		if (pController->GetButton(knArmHigh))//4
		{
			m_fTargetArmAngle = 495.0f;
			
			// TODO: if new setpoint below old setpoint, change PID values
			
		}
		else if (pController->GetButton(knArmMiddle))//3
		{
			m_fTargetArmAngle = 655.0f;
		}
		else if (pController->GetButton(knArmLow))//2
		{
			m_fTargetArmAngle = 847.0f;
		}
		
		// give the joystick the opportunity to over-ride
		float fArmControl = pController->GetAxis(knArmUpDown);
		if (fabs(fArmControl) > 0.05)	// controller over-ride
		{
			m_fArmSpeed = kArmGain*fArmControl;
			
			// This is necessary so that the arm will think it is at
			// the "right" angle wherever it winds up
			m_fTargetArmAngle = m_mapAnalogSensors[knArmAngle];
		}
		else	// no joystick signal, let PID do its thing
		{
#ifdef PID_ARM
			m_pArmPID->setDesiredValue(m_fTargetArmAngle);
			m_fArmSpeed = kArmGain*m_pArmPID->calcPID(m_mapAnalogSensors[knArmAngle]);
#endif
		}

		if (fabs(pController->GetAxis(knRollInOut))<0.05)
		{
			m_fLowerJawRollerSpeed = pController->GetAxis(knRollInOut);
			m_fUpperJawRollerSpeed = -pController->GetAxis(knRollInOut);
		}
		else
		{
			m_fLowerJawRollerSpeed = pController->GetAxis(knRollInOut)+pController->GetAxis(knRollAround);
			m_fUpperJawRollerSpeed = -pController->GetAxis(knRollInOut)+pController->GetAxis(knRollAround);
		}
		if (pController->GetAxis(knRollInOut) > 0)
		{
			if ((m_mapDigitalSensors[knTubeRight] == 0) && (m_mapDigitalSensors[knTubeLeft] == 0))
			{
				m_fLowerJawRollerSpeed = (m_mapAnalogSensors[knTubeIR]-300.0)/200.0;
				m_fUpperJawRollerSpeed = -m_fLowerJawRollerSpeed;
			}
		}
	}
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
	
	m_mapWeightX[knLineFollowing] = 0.0f; // assume no input from line
	m_mapWeightY[knLineFollowing] = 0.0f;
	m_mapWeightR[knLineFollowing] = 0.0f;
	
	m_mapWeightX[knWallAlign] = 0.0f; // assume no input from wall
	m_mapWeightY[knWallAlign] = 0.0f;
	m_mapWeightR[knWallAlign] = 0.0f;

	// re-init stopped count so we can use line follow again
	if (!pController->GetButton(knLineFollowButton))
	{
		m_nStoppedCount = -1;
	}
	
	if (fabs(m_mapR[knDriverInput]) > 0.1f)	// let stick have control
	{
		m_fGyroSetPoint = m_mapAnalogSensors[knGyro];
	}
	else if (pController->GetButton(knLineFollowButton))
	{
		if (-1 == m_nStoppedCount)
		{
			m_nStoppedCount = 0;
		}
		m_mapWeightR[knLineFollowing] = 1.0f;	// consider line rotation
		m_mapWeightY[knLineFollowing] = 1.0f;
		m_mapWeightR[knWallAlign] = 0.0f;	// Consider wall rotation
		m_mapWeightY[knWallAlign] = 0.0f;	// let wall reduce speed
		m_mapWeightR[knDriverInput] = 0.0f;	// ignore driver rotation	
		m_mapWeightY[knDriverInput] = 0.0f;	// ignore stick
	}
	else						// let gyro have control
	{
		if (fabs(m_mapR[knGyroTracking]-m_fGyroSetPoint) < 15.0f)
		{
			m_mapWeightR[knGyroTracking] = 1.0f;
			m_mapWeightR[knDriverInput] = 0.0f;
		}
		else // something bad has happened!  Rezero gyro
		{
			m_fGyroSetPoint = m_mapAnalogSensors[knGyro];			
		}
	}	
}

void KBot::UpdateMotors()
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
	
	double wheelSpeeds[4];  // signs changed to get rotation right
	wheelSpeeds[0] = -fX + fY - fR;
	wheelSpeeds[1] = fX + fY + fR;
	wheelSpeeds[2] = -fX + fY + fR;
	wheelSpeeds[3] = fX + fY - fR;
	
	DeadbandNormalize(wheelSpeeds);

	// actually set speeds (negate right side due to motor mounting)
	//UINT8 syncGroup = 0x80;	
	m_pLeftFrontJaguar->Set(wheelSpeeds[0]*kfDriveJaguarConstant);// , syncGroup);
	Wait(0.001);
	m_pRightFrontJaguar->Set(-wheelSpeeds[1]*kfDriveJaguarConstant);// , syncGroup);
	Wait(0.001);
	m_pRightBackJaguar->Set(-wheelSpeeds[2]*kfDriveJaguarConstant);//, syncGroup);
	Wait(0.001);
	m_pLeftBackJaguar->Set(wheelSpeeds[3]*kfDriveJaguarConstant);// , syncGroup);
	Wait(0.001);
	m_pLowerRollerJaguar->Set(10*m_fLowerJawRollerSpeed);// , syncGroup);
	Wait(0.001);
	m_pUpperRollerJaguar->Set(10*m_fUpperJawRollerSpeed);// , syncGroup);
	Wait(0.001);
	m_pArmJaguar->Set(m_fArmSpeed);//, syncGroup);
	//CANJaguar::UpdateSyncGroup(syncGroup);	
}

void KBot::UpdateWrist()
{
	if (m_nWristPosition == 1)	// wrist OUT
	{
		m_pWristInSolenoid->Set(true);
		m_pWristOutSolenoid->Set(false);
	}
	else	// wrist IN
	{
		m_pWristInSolenoid->Set(false);
		m_pWristOutSolenoid->Set(true);		
	}
}

void KBot::UpdateJaw()
{
	if (m_nJawPosition == 0)	// Jaw CLOSED
	{
		m_pJawClosedSolenoid->Set(true);
		m_pJawOpenSolenoid->Set(false);		
	}
	else	// Jaw OPEN
	{
		m_pJawClosedSolenoid->Set(false);
		m_pJawOpenSolenoid->Set(true);				
	}
}

void KBot::UpdateDeployer()
{
	if (m_nDeployerPosition == 0)	// Deployer IN
	{
		m_pDeployerInSolenoid->Set(true);
		m_pDeployerOutSolenoid->Set(false);		
	}
	else	// Deployer OUT
	{
		m_pDeployerInSolenoid->Set(false);
		m_pDeployerOutSolenoid->Set(true);		
	}
}

void KBot::UpdateActuators()
{
	UpdateMotors();
	UpdateWrist();
	UpdateJaw();
	UpdateDeployer();
	UpdateLights();
}

void KBot::UpdateLights()
{
	if (knAllLightsOff == m_nLightState)
	{
		m_pRedLightRelay->Set(Relay::kOff);
		m_pBlueLightRelay->Set(Relay::kOff);
		m_pWhiteLightRelay->Set(Relay::kOff);
	}
	else if (knRedLight == m_nLightState)
	{
		m_pRedLightRelay->Set(Relay::kForward);
		m_pBlueLightRelay->Set(Relay::kOff);
		m_pWhiteLightRelay->Set(Relay::kOff);
	}
	else if (knBlueLight == m_nLightState)
	{
		m_pRedLightRelay->Set(Relay::kOff);
		m_pBlueLightRelay->Set(Relay::kForward);
		m_pWhiteLightRelay->Set(Relay::kOff);
	}
	else if (knWhiteLight == m_nLightState)
	{
		m_pRedLightRelay->Set(Relay::kOff);
		m_pBlueLightRelay->Set(Relay::kOff);
		m_pWhiteLightRelay->Set(Relay::kForward);
	}
	else if (m_nLightState == knAllLightsOn)
	{
		m_pRedLightRelay->Set(Relay::kForward);
		m_pBlueLightRelay->Set(Relay::kForward);
		m_pWhiteLightRelay->Set(Relay::kForward);
	}
	else if (m_nLightState == knTubeCaptureSignal)
	{
		m_pRedLightRelay->Set(Relay::kForward);
		m_pBlueLightRelay->Set(Relay::kForward);
		m_pWhiteLightRelay->Set(Relay::kOff);		
	}
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

