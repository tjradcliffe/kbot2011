#include "programmed_controller.h"

// require mappings so we know what axis to mirror
#include "mappings.h"

// gotta know about the robot
#include "kbot.h"

// standard includes
#include <iostream>

ProgrammedController::ProgrammedController(KBot* pRobot) :
	m_pRobot(pRobot), m_nAutoState(0), m_nAutoCount(0)
{
}

ProgrammedController::~ProgrammedController()
{
	// do nothing
}

void ProgrammedController::Reset()
{
	Controller::Reset();
	m_nAutoState = 0;
	m_nAutoCount = 0;
}
	
void ProgrammedController::Update()
{
	switch (m_nAutoState)
	{
	case 0:
	{
		m_nAutoState++;
	}
	case 1:							// Feed tube in
	{
		m_vecAxes[knX] = 0.0;
		m_vecAxes[knY] = 0.0;
		m_vecAxes[knR] = 0.0;
		m_vecAxes[knRollInOut] = -0.75;
		if (++m_nAutoCount >= m_pRobot->kPeriodicSpeed*0.5)	// For 0.5 seconds
		{
			m_nAutoState++;
			m_nAutoCount=0;
		}
	}	
	case 2:							// Drive forward at 3/4 speed
	{								// while lifting arm
									// and using line following
									// and stopping a set distance from the wall
		m_vecAxes[knX] = 0.0;
		m_vecAxes[knY] = -0.75;
		m_vecAxes[knR] = 0.0;
		m_vecAxes[knRollInOut] = 0.0;
		
		// Line following adjustment:
		float fSignal = 0;
		if (0 == m_pRobot->m_mapDigitalSensors[knLineRight])
		{
			fSignal += 0.25;
		}
		if (0 == m_pRobot->m_mapDigitalSensors[knLineLeft])
		{
			fSignal -= 0.25;		
		}
		m_vecAxes[knR] = fSignal;
		
		// Distance correction
		static float targetDist = 300.0;		// 3 meters
		static float startSlowingDist = 500.0;	// 5 meters
		static float closeEnough = 5.0;
		
		float dist = (m_pRobot->m_mapAnalogSensors[knLeftIRSensor]+m_pRobot->m_mapAnalogSensors[knRightIRSensor])/2.0;
		float gain = (dist-targetDist)/(startSlowingDist-targetDist);
		// TODO: TRY: gain = (1-(1-gain)*(1-gain)); // Squared to make dropoff slower at start, faster at end to minimize running motor at stall voltage
		if (dist<startSlowingDist)
		{
			for (int i=0; i<4; i++)
			{
				if (fabs(dist-targetDist) < closeEnough)
				{
					// TODO: wheelSpeeds[i] = 0.0;
				} else
				{
					// TODO: wheelSpeeds[i] *= gain;
				}
			}
		}

		m_vecAxes[knArmUpDown] = 0.5; // TOTAL GUESS!!!
		if (++m_nAutoCount >= m_pRobot->kPeriodicSpeed*5.7)	// For 5.7 seconds
		{
			m_nAutoState++;
			m_nAutoCount=0;
		}
	}
	case 3:							// Rotate tube forward
	{
		// TODO: Remove file output
		std::string outputFilename = "DistanceFromWall.txt";
		std::ofstream *m_pOutStream = 0;
		if (0 != outputFilename.size())
		{
			m_pOutStream = new std::ofstream(outputFilename.c_str());
		}
		if (0 != m_pOutStream)
		{
			(*m_pOutStream) << m_pRobot->m_mapAnalogSensors[knLeftIRSensor] << ", " << m_pRobot->m_mapAnalogSensors[knRightIRSensor] << std::endl;
		}
		delete m_pOutStream;

		m_vecAxes[knX] = 0.0f;
		m_vecAxes[knY] = 0.0f;
		m_vecAxes[knR] = 0.0f;
		m_vecAxes[knRollInOut] = 0.75;
		m_vecAxes[knArmUpDown] = 0;
		if (++m_nAutoCount >= m_pRobot->kPeriodicSpeed*0.5)	// For 0.5 seconds
		{
			m_nAutoState++;
			m_nAutoCount=0;
		}
	}
	case 4:							// Open jaw
	{
		m_vecAxes[knX] = 0.0f;
		m_vecAxes[knY] = 0.0f;
		m_vecAxes[knR] = 0.0f;
		m_vecAxes[knRollInOut] = 0.75;
		m_vecAxes[knArmUpDown] = 0;
		m_vecButtons[knJawOpen] = 1;
		if (++m_nAutoCount >= m_pRobot->kPeriodicSpeed*0.5)	// For 0.5 seconds
		{
			m_nAutoState++;
			m_nAutoCount=0;
		}
	}
	case 5:							// Turn away
	{
		m_vecAxes[knX] = 0.0f;
		m_vecAxes[knY] = 0.5f;
		m_vecAxes[knR] = 0.25f;
		m_vecAxes[knRollInOut] = 0;
		m_vecAxes[knArmUpDown] = 0;
		m_vecButtons[knJawOpen] = 0;
		if (++m_nAutoCount >= m_pRobot->kPeriodicSpeed*0.8)	// For 0.8 seconds
		{
			m_nAutoState++;
			m_nAutoCount=0;
		}
	}
	default:						// Stop everything!!
	{
		m_vecAxes[knX] = 0;
		m_vecAxes[knY] = 0;
		m_vecAxes[knR] = 0;
		m_vecAxes[knRollInOut] = 0;
		m_vecAxes[knArmUpDown] = 0;
		m_vecButtons[knJawOpen] = 0;
	}
	}
}
