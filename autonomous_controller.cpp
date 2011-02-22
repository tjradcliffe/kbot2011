#include "autonomous_controller.h"

// require mappings so we know what axis to mirror
#include "mappings.h"

// standard includes
#include <iostream>

AutonomousController::AutonomousController(KBot* pRobot) :
	m_pRobot(pRobot), m_bResetStream(false), m_bMirror(false)
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
		pInStream = new std::ifstream(m_strFilename.c_str());
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
		
		if (m_bMirror)	// mirror the rotation axis
		{
			m_vecAxes[knR] *= -1;
		}
	}
	else	// file did not open or ran out
	{
		for (int nIndex = 0; nIndex < m_nStickNumber*m_nAxisNumber; ++nIndex)
		{
			m_vecAxes[nIndex] = 0;
		}		
	}
	
	++m_nTimeCount;
	
}

void AutonomousController::Reset()
{
	Controller::Reset();
	m_bResetStream = true;
}
