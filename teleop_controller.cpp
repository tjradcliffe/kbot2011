#include "teleop_controller.h"

#include "JoyStick.h"

// standard includes
#include <iostream>

TeleopController::TeleopController(std::string strOutputFilename)
{
	// xbox controller appears as single USB joystick
	m_pStick = new Joystick(2);
	m_strOutputFilename = strOutputFilename;

	m_pOutStream = 0;
}

TeleopController::~TeleopController()
{
	delete m_pStick;
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
		m_vecAxes[nIndex] = m_pStick->GetRawAxis(nIndex+1);
	}
	for (int nIndex = 0; nIndex < m_nButtonNumber; ++nIndex)
	{
		m_vecButtons[nIndex] = m_pStick->GetRawButton(nIndex+1);
	}

	if (0 != m_pOutStream)
	{
		(*m_pOutStream) << m_nTimeCount << " ";
		for (int nIndex = 0; nIndex < m_nAxisNumber; ++nIndex)
		{
			(*m_pOutStream) << m_vecAxes[nIndex] << " ";
		}
		for (int nIndex = 0; nIndex < m_nButtonNumber; ++nIndex)
		{
			(*m_pOutStream) << m_vecButtons[nIndex] << " ";
		}
		(*m_pOutStream) << std::endl;
	}
	++m_nTimeCount;
}

