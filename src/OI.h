#pragma once

#include <WPILib.h>

// ==========================================================================

class OI {
private:
	Joystick* driverJoystick;
	Joystick* driverJoystick2;


public:
	OI();

	float GetRightTrigger();
	float GetLeftTrigger();
	float GetJoystickX();
	float GetJoystickX2();
	float GetJoystickY2();
	float GetJoystickY();
	float GetJoystickZ();
	bool GetButtonStart();
	bool GetButtonA();
	bool GetButtonTrig();
	bool GetButton2();
	bool GetButton3();
	bool GetButton4();
	bool GetButton5();

	Command* spinIndexerCW;
	Command* spinIndexerCCW;
	Command* pickupIn;
	Command* pickupOut;
	Command* unwindWheels;
	Command* winchSet1;
	Command* winchSet2;
	Command* winchSet3;
	//Command* arcade;
	Command* climb;
	Command* climbReverse;
	Command* gyroCrab;
	Command* fieldCentric;

	Joystick* GetDriverJoystick() { return driverJoystick; }
	Joystick* GetDriverJoystick2() { return driverJoystick2; }
};

// ==========================================================================
