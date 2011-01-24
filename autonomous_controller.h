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
	
	AutonomousController(KBot* pRobot, std::string strInputFilename);
	~AutonomousController();
	
	void Update();
	
	void Reset();

private:

	KBot *m_pRobot;
	std::string m_strInputFilename;	
	bool m_bResetStream;
};

#endif // TELEOP_CONTROLLER_H
