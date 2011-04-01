#ifndef PROGRAMMED_CONTROLLER_H
#define PROGRAMMED_CONTROLLER_H

// local includes
#include "controller.h"

// local forward declarations
class KBot;

// standard includes
#include <fstream>
#include <string>

class ProgrammedController : public Controller
{
public:
	
	ProgrammedController(KBot* pRobot);
	~ProgrammedController();

	//! Update the state from the file
	void Update();
	
	//! Zero state and count
	void Reset();
	
private:

	//! Give us access to the robot stat
	KBot *m_pRobot;
	
	int m_nAutoState;
	int m_nAutoCount;	
};

#endif // PROGRAMMED_CONTROLLER_H
