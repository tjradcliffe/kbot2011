#ifndef AUTONOMOUS_CONTROLLER_H
#define AUTONOMOUS_CONTROLLER_H

// local includes
#include "controller.h"

// local forward declarations
class KBot;

// standard includes
#include <fstream>
#include <string>

class AutonomousController : public Controller
{
public:
	
	AutonomousController(KBot* pRobot);
	~AutonomousController();

	//! Update the state from the file
	void Update();
	
	//! Reset to start of stream
	void Reset();
	
	//! Set the mirror flag
	void SetMirror(bool bMirror) 
	{
		m_bMirror = bMirror;
	}

private:

	//! Give us access to the robot stat
	KBot *m_pRobot;
	
	//! If true, delete and recreate the stream, open at start
	bool m_bResetStream;
	
	//! If true, change the sign on the rotations to mirror the recording (default false)
	bool m_bMirror;
};

#endif // AUTONOMOUS_CONTROLLER_H
