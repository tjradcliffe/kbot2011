#ifndef CONTROLLER_H
#define CONTROLLER_H

// standard includes
#include <vector>

// local includes
#include "mappings.h"

/*!
Pure virtual base class for the system controller,
which abstrcts the operator control device (joysticks etc).

Different derivived classes will distinguish teleop and auto.
*/
class Controller
{
public:
	
	//! Constructor sets up axes and zeros time count
	Controller() 
	{
		m_nStickNumber = 2;	// we have two "joysticks" as input

		m_nAxisNumber = knAxes;	// make room for axis values (per stick)
		m_vecAxes.resize(m_nStickNumber*m_nAxisNumber);

		m_nButtonNumber = knButtons; // make room for button values (per stick)
		m_vecButtons.resize(m_nStickNumber*m_nButtonNumber);
		
		m_nTimeCount = 0;
	}
	
	//! Constructor just defined to be virtual
	virtual ~Controller() {;}
	
	//! Zero the time count
	virtual void Reset()
	{
		m_nTimeCount = 0;
	}
	
	//@{
	//! Get the controller state out of the arrays
	int GetButton(int nButtonIndex) {return m_vecButtons[nButtonIndex];}
	float GetAxis(int nAxisIndex) {return m_vecAxes[nAxisIndex];}
	//@}

	//! Update the arrays from hardware etc
	virtual void Update() = 0;
	
protected:
	
	//! The button states
	std::vector<int> m_vecButtons;
	
	//! The axis states
	std::vector<float> m_vecAxes;
	
	//! Number of axes per joystick
	int m_nAxisNumber;
	
	//! Number of buttons per joystick
	int m_nButtonNumber;

	//! The number of joysticks 
	int m_nStickNumber;
	
	//! The number of times update has been called
	int m_nTimeCount;
};

#endif // CONTROLLER_H
