#include "teleop_controller.h"

#include "JoyStick.h"

// standard includes
#include <iostream>

static int knOperatorStick = 1;
static int knDriverStick = 2;

TeleopController::TeleopController(std::string strOutputFilename)
{
	// xbox controller appears as single USB joystick
	m_pDriverStick = new Joystick(knDriverStick);
	m_pOperatorStick = new Joystick(knOperatorStick);
	
	m_strOutputFilename = strOutputFilename;

	m_pOutStream = 0;
}

TeleopController::~TeleopController()
{
	delete m_pOperatorStick;
	delete m_pDriverStick;
	delete m_pOutStream;
}

void TeleopController::Done()
{
	if (0 != m_pOutStream)
	{
		m_pOutStream->flush();
		m_pOutStream->close();
	}
	delete m_pOutStream;
}

void TeleopController::Reset()
{
	Controller::Reset();
	delete m_pOutStream;
	if (0 < m_strOutputFilename.size())
	{
		m_pOutStream = new std::ofstream(m_strOutputFilename.c_str());
	}
	else
	{
		m_pOutStream = 0;
	}
}

void TeleopController::Update()
{
	for (int nIndex = 0; nIndex < m_nAxisNumber; ++nIndex)
	{
		m_vecAxes[nIndex] = m_pDriverStick->GetRawAxis(nIndex+1);
	}
	for (int nIndex = 0; nIndex < m_nAxisNumber; ++nIndex)
	{
		m_vecAxes[m_nAxisNumber+nIndex] = m_pOperatorStick->GetRawAxis(nIndex+1);
	}
	for (int nIndex = 0; nIndex < m_nButtonNumber; ++nIndex)
	{
		m_vecButtons[nIndex] = m_pDriverStick->GetRawButton(nIndex+1);
	}
	for (int nIndex = 0; nIndex < m_nButtonNumber; ++nIndex)
	{
		m_vecButtons[m_nButtonNumber+nIndex] = m_pOperatorStick->GetRawButton(nIndex+1);
	}

	if (0 != m_pOutStream)
	{
		(*m_pOutStream) << m_nTimeCount << " ";
		for (int nIndex = 0; nIndex < m_nStickNumber*m_nAxisNumber; ++nIndex)
		{
			(*m_pOutStream) << m_vecAxes[nIndex] << " ";
		}
		for (int nIndex = 0; nIndex < m_nButtonNumber; ++nIndex)
		{
			(*m_pOutStream) << m_nStickNumber*m_vecButtons[nIndex] << " ";
		}
		(*m_pOutStream) << std::endl;
	}
	++m_nTimeCount;
}

