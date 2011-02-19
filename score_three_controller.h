#ifndef SCORE_THREE_CONTROLLER_H
#define SCORE_THREE_CONTROLLER_H

// local includes
#include "controller.h"

// local forward declarations
class KBot;

class ScoreThreeController : public Controller
{
public:
	
	ScoreThreeController(KBot* pKBot);
	
	~ScoreThreeController();
	
	void Update();
	
	void Reset();

private:

	KBot *m_pRobot;
};

#endif	// SCORE_THREE_CONTORLLER_H
