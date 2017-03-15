#include <Commands/BoilerLineup.h>
#include <Commands/BoilerLineup.h>
#include "OI.h"
#include <SmartDashboard/SmartDashboard.h>
#include "Commands/SetWheelOffsets.h"
#include "Commands/ArcadeDriveMode.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/BasicCameraEnableCmd.h"
#include "Commands/FieldCentric.h"
#include "Commands/GyroCrab.h"
#include "Commands/ScriptCamDrive.h"
#include "Commands/ScriptValidate.h"
#include "Commands/SetWheelOffsets.h"
#include "Commands/UnwindWheels.h"
#include "Commands/UpdatePositions.h"
#include "Commands/ZeroYaw.h"
#include "Commands/SpinIndexerCW.h"
#include "Commands/SpinIndexerCCW.h"
#include "Commands/PickupIn.h"
#include "Commands/PickupOut.h"
#include <Commands/ShooterRun.h>
#include <Commands/ShooterFaster.h>
#include <Commands/ShooterSlower.h>
#include <Commands/OneBall.h>
#include <Commands/TestJamShooter.h>
#include "Commands/Climb.h"
#include "Commands/ReverseClimb.h"
#include "Commands/GearLight.h"
#include "Commands/FuelLight.h"
#include "Commands/ZeroReverse.h"
#include "Commands/ScriptShootWithVision.h"
#include "Commands/GearLineup.h"
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

const uint32_t JOYSTICK_BUTTON_TRIG = 1;
const uint32_t JOYSTICK_BUTTON_2 = 2;
const uint32_t JOYSTICK_BUTTON_3 = 3;
const uint32_t JOYSTICK_BUTTON_4 = 4;
const uint32_t JOYSTICK_BUTTON_5 = 5;
const uint32_t JOYSTICK_BUTTON_6 = 6;
const uint32_t JOYSTICK_BUTTON_7 = 7;
const uint32_t JOYSTICK_BUTTON_10 = 10;

// ==========================================================================

OI::OI() {
	spinIndexerCW = new SpinIndexerCW();
	spinIndexerCCW = new SpinIndexerCCW();
	gearLight = new GearLight();
	fuelLight = new FuelLight();
	pickupIn = new PickupIn();
	pickupOut = new PickupOut();
	driverJoystick = new Joystick(0);
	driverJoystick2 = new Joystick(1);
	unwindWheels = new UnwindWheels();
	gyroCrab = new GyroCrab();
	fieldCentric = new FieldCentric();
	//arcade = new ArcadeDriveMode();
	oneBall = new OneBall();
	climb = new Climb();
	zeroReverse = new ZeroReverse();
	autoShoot = new ScriptShootWithVision(3000, 10);
	gearLineup = new GearLineup();
	//reverseClimb = new ReverseClimb();
	auto cameraEnableCmd = new BasicCameraEnableCmd(Robot::basicCameraSub);


////Main Driver Controller///----------------------------------------------------------------
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_LB))->WhileHeld(gyroCrab);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_RB))->WhileHeld(fieldCentric);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_START))->WhileHeld(climb);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_X))->WhileHeld(autoShoot);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_B))->WhileHeld(gearLineup);

	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_Y))->ToggleWhenPressed(gearLight);
	(new JoystickButton(driverJoystick, JOYSTICK_BUTTON_A))->ToggleWhenPressed(fuelLight);
////-----------------------------------------------------------------------------------------

////Secondary Driver Controller///-----------------------------------------------------------
	(new JoystickButton(driverJoystick2, JOYSTICK_BUTTON_RB))->WhileHeld(spinIndexerCW);
	(new JoystickButton(driverJoystick2, JOYSTICK_BUTTON_LB))->WhileHeld(spinIndexerCCW);
	(new JoystickButton(driverJoystick2, JOYSTICK_BUTTON_Y))->WhenPressed(oneBall);

	auto cmdShooterRun = new ShooterRun();
	auto shooterRunButton = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_A);
	shooterRunButton->WhileHeld(cmdShooterRun);

	auto cmdShooterSlower = new ShooterSlower();
	auto shooterSlowerButton = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_X);
	shooterSlowerButton->WhenPressed(cmdShooterSlower);

	//auto shooterSlowerButton2 = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_2);
	//shooterSlowerButton2->WhenPressed(cmdShooterSlower);

	auto cmdShooterFaster = new ShooterFaster();
	auto shooterFasterButton = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_B);
	shooterFasterButton->WhenPressed(cmdShooterFaster);

	//auto shooterFasterButton2 = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_3);
	//shooterFasterButton2->WhenPressed(cmdShooterFaster);
////-----------------------------------------------------------------------------------------

	SmartDashboard::PutData("Camera On", cameraEnableCmd);

	//SmartDashboard::PutData("Camera", useCamera);
	SmartDashboard::PutData("SetWheelOffsets", new SetWheelOffsets());
	SmartDashboard::PutData("Zero Yaw", new ZeroYaw());

	SmartDashboard::PutData("Update Positions", new UpdatePositions());
	SmartDashboard::PutData("Validate Script", new ScriptValidate());
	//SmartDashboard::PutData("JAM SHOOTER", new TestJamShooter());
	SmartDashboard::PutData("Zero Reverse Timer", zeroReverse);
}

bool OI::GetButton2() {
  auto value = GetDriverJoystick2()->GetRawButton(JOYSTICK_BUTTON_2);
  return (value);
}

//==========================================================================

bool OI::GetButton6() {
	auto value = GetDriverJoystick2()->GetRawButton(JOYSTICK_BUTTON_6);
	return (value);
}

//==========================================================================


bool OI::GetButton10() {
  auto value = GetDriverJoystick2()->GetRawButton(JOYSTICK_BUTTON_10);
  return (value);
}

//==========================================================================

bool OI::GetButton5() {
  auto value = GetDriverJoystick2()->GetRawButton(JOYSTICK_BUTTON_5);
  return (value);
}

//==========================================================================

bool OI::GetButtonTrig() {
  auto value = GetDriverJoystick2()->GetRawButton(JOYSTICK_BUTTON_TRIG);
  return (value);
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

float OI::GetJoystickX2() {
	auto value = driverJoystick2->GetRawAxis(JOYSTICK_LX_AXIS);
	return (fabs(value) < JOYSTICK_DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickY2() {
	auto value = driverJoystick2->GetRawAxis(JOYSTICK_LY_AXIS);
	return (fabs(value) < JOYSTICK_DEAD_ZONE) ? 0 : value;
}

//============================================================================

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


//saving these controls down here in case we switch back at some point
/*
////Secondary Driver Controller///-----------------------------------------------------------
	(new JoystickButton(driverJoystick2, JOYSTICK_BUTTON_6))->WhileHeld(spinIndexerCW);
	(new JoystickButton(driverJoystick2, JOYSTICK_BUTTON_10))->WhileHeld(spinIndexerCCW);
	(new JoystickButton(driverJoystick2, JOYSTICK_BUTTON_7))->WhenPressed(oneBall);

	auto cmdShooterRun = new ShooterRun();
	auto shooterRunButton = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_TRIG);
	shooterRunButton->WhileHeld(cmdShooterRun);

	auto cmdShooterSlower = new ShooterSlower();
	auto shooterSlowerButton = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_4);
	shooterSlowerButton->WhileHeld(cmdShooterSlower);

	auto shooterSlowerButton2 = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_2);
	shooterSlowerButton2->WhenPressed(cmdShooterSlower);

	auto cmdShooterFaster = new ShooterFaster();
	auto shooterFasterButton = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_5);
	shooterFasterButton->WhileHeld(cmdShooterFaster);

	auto shooterFasterButton2 = new frc::JoystickButton(driverJoystick2, JOYSTICK_BUTTON_3);
	shooterFasterButton2->WhenPressed(cmdShooterFaster);
////-----------------------------------------------------------------------------------------
*/
