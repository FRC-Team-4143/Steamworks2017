#include "OI.h"
#include <SmartDashboard/SmartDashboard.h>
#include "Commands/SetWheelOffsets.h"
#include "Commands/ArcadeDriveMode.h"
#include "Commands/ArmDown.h"
#include "Commands/ArmUp.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/BasicCameraEnableCmd.h"
#include "Commands/Climb.h"
#include "Commands/ClimbReverse.h"
#include "Commands/DeFeed.h"
#include "Commands/FieldCentric.h"
#include "Commands/GyroCrab.h"
#include "Commands/LowShot.h"
#include "Commands/PositionDrive.h"
#include "Commands/ResetArm.h"
#include "Commands/ResetWinch.h"
#include "Commands/ScriptCamDrive.h"
#include "Commands/ScriptValidate.h"
#include "Commands/SetWheelOffsets.h"
#include "Commands/SetWinchPosition.h"
#include "Commands/ShootCycle.h"
#include "Commands/ShootCycle.h"
#include "Commands/StowArm.h"
#include "Commands/UnwindWheels.h"
#include "Commands/UpdatePositions.h"
#include "Commands/ZeroYaw.h"
#include "Robot.h"

const uint32_t JOYSTICK_LX_AXIS = 0;
const uint32_t JOYSTICK_LY_AXIS = 1;
const uint32_t JOYSTICK_LTRIG_AXIS = 2;
const uint32_t JOYSTICK_RTRIG_AXIS = 3;
const uint32_t JOYSTICK_RX_AXIS = 4;
const uint32_t JOYSTICK_RY_AXIS = 5;

const uint32_t JOYSTICK_BUTTON_A = 1;
const uint32_t JOYSTICK_BUTTON_B = 2;
const uint32_t JOYSTICK_BUTTON_X = 3;
const uint32_t JOYSTICK_BUTTON_Y = 4;
const uint32_t JOYSTICK_BUTTON_LB = 5;
const uint32_t JOYSTICK_BUTTON_RB = 6;
const uint32_t JOYSTICK_BUTTON_BACK = 7;
const uint32_t JOYSTICK_BUTTON_START = 8;
const uint32_t JOYSTICK_BUTTON_LEFT = 9;
const uint32_t JOYSTICK_BUTTON_RIGHT = 10;
const float JOYSTICK_DEAD_ZONE = 0.1;

// ==========================================================================

OI::OI() {
	driverJoystick = new Joystick(0);
	armUp = new ArmUp();
	armDown = new ArmDown();
	stowArm = new StowArm();
	unwindWheels = new UnwindWheels();
	winchSet1 = new SetWinchPosition(0, false, 1.5); // starting
	winchSet2 = new SetWinchPosition(1, false, 2.5); // raised
	winchSet3 = new SetWinchPosition(2, false, 1.5); // lowered
	deFeed = new DeFeed(1);
	gyroCrab = new GyroCrab();
	fieldCentric = new FieldCentric();
	shootCycle = new ShootCycle(0);
	lowShot = new LowShot();
	//arcade = new ArcadeDriveMode();
	climb = new Climb();
	climbReverse = new ClimbReverse();

	auto cameraEnableCmd = new BasicCameraEnableCmd(Robot::basicCameraSub);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_Y))->WhileHeld(armDown);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_X))->WhileHeld(stowArm);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_B))->WhileHeld(climbReverse);//->WhileHeld(lowShot);


	//(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_BACK))->WhenPressed(winchSet2);
	//(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_START))->WhenPressed(winchSet3);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_START))->WhenPressed(cameraEnableCmd);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_RB))->WhileHeld(unwindWheels);

	//(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_START))->WhileHeld(arcade);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_LB))->WhenPressed(deFeed);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_A))->WhileHeld(climb);
	//->WhileHeld(shootCycle);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_RIGHT))->WhileHeld(gyroCrab);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_LEFT))->WhileHeld(fieldCentric);

	SmartDashboard::PutData("Camera On", cameraEnableCmd);

	//SmartDashboard::PutData("Camera", useCamera);
	SmartDashboard::PutData("SetWheelOffsets", new SetWheelOffsets());
	SmartDashboard::PutData("Zero Yaw", new ZeroYaw());

	SmartDashboard::PutNumber("Winch 0", 2.99);
	SmartDashboard::PutNumber("Winch 1", 2.49);
	SmartDashboard::PutNumber("Winch 2", 2.99);

	SmartDashboard::PutData("Update Positions", new UpdatePositions());
	SmartDashboard::PutData("Validate Script", new ScriptValidate());
}

// ==========================================================================

float OI::GetRightTrigger() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_RTRIG_AXIS);
	return (fabs(value) < JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetLeftTrigger() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_LTRIG_AXIS);
	return (fabs(value) < JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickX() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_LX_AXIS);
	return (fabs(value) < JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickY() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_LY_AXIS);
	return (fabs(value) < JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickZ() {
	auto value = driverJoystick->GetRawAxis(JOYSTICK_RX_AXIS);
	return (fabs(value) < JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

bool OI::GetButtonStart() {
	auto value = GetDriverJoystick()->GetRawButton(JOYSTICK_BUTTON_START);
	return value;
}

// ==========================================================================

bool OI::GetButtonA() {
	auto value = GetDriverJoystick()->GetRawButton(JOYSTICK_BUTTON_A);
	return value;
}

// ==========================================================================
