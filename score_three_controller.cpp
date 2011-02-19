
#include "score_three_controller.h"

// local includes
#include "kbot.h"

ScoreThreeController::ScoreThreeController(KBot* pRobot):
	m_pRobot(pRobot)
{
}

ScoreThreeController::~ScoreThreeController()
{
	// do nothing
}
	
void ScoreThreeController::Update()
{
}

void ScoreThreeController::Reset()
{
	Controller::Reset();
}
