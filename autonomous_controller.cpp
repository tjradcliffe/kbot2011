#include "autonomous_controller.h"

// standard includes
#include <iostream>

AutonomousController::AutonomousController(KBot* pRobot, std::string strInputFilename) :
	m_pRobot(pRobot), m_strInputFilename(strInputFilename), m_bResetStream(false)
{
}

AutonomousController::~AutonomousController()
{
	// do nothing
}
	
void AutonomousController::Update()
{
	static std::ifstream* pInStream = 0;

	if (m_bResetStream)
	{
		delete pInStream;
		pInStream = 0;
		m_bResetStream = false;
	}
	
	if (0 == pInStream)
	{
		pInStream = new std::ifstream(m_strInputFilename.c_str());
	}
	
	int nTimeCount = 0;
	if (!(pInStream->fail()))
	{
		(*pInStream) >> nTimeCount;
		for (int nIndex = 0; nIndex < m_nStickNumber*m_nAxisNumber; ++nIndex)
		{
			(*pInStream) >> m_vecAxes[nIndex];
		}
		for (int nIndex = 0; nIndex < m_nStickNumber*m_nButtonNumber; ++nIndex)
		{
			(*pInStream) >> m_vecButtons[nIndex];
		}
	}
	
	++m_nTimeCount;
	
}

void AutonomousController::Reset()
{
	Controller::Reset();
	m_bResetStream = true;
}
