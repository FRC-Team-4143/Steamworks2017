#pragma once

#include <WPILib.h>

// ==========================================================================

class OI {
private:
	Joystick* driverJoystick;

public:
	OI();

	float GetRightTrigger();
	float GetLeftTrigger();
	float GetJoystickX();
	float GetJoystickY();
	float GetJoystickZ();
	bool GetButtonStart();
	bool GetButtonA();

	Command* armUp;
	Command* armDown;
	Command* stowArm;
	Command* unwindWheels;
	Command* winchSet1;
	Command* winchSet2;
	Command* winchSet3;
	Command* deFeed;
	//Command* arcade;
	Command* climb;
	Command* climbReverse;
	Command* gyroCrab;
	Command* shootCycle;
	Command* fieldCentric;
	Command* lowShot;

	Joystick* GetDriverJoystick() { return driverJoystick; }
};

// ==========================================================================
