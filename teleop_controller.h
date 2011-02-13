#ifndef TELEOP_CONTROLLER_H
#define TELEOP_CONTROLLER_H

// local includes
#include "controller.h"

// local forward declarations
class KBot;
class Joystick;

// standard includes
#include <fstream>
#include <string>

class TeleopController : public Controller
{
public:
	
	TeleopController(std::string strOutputFilename = "");
	~TeleopController();
	
	void Update();
	
	void Reset();
	
	void Done();

private:
	
	//! the two joysticks being used (actually xbox controllers)
	Joystick *m_pDriverStick;
	Joystick *m_pOperatorStick;

	//! output stream for recording
	std::ofstream *m_pOutStream;
	
	//! Output stream name
	std::string m_strOutputFilename;
};

#endif // TELEOP_CONTROLLER_H
