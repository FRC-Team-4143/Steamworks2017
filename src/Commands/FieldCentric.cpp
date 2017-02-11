#include "Commands/FieldCentric.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

FieldCentric::FieldCentric()
: frc::Command("FieldCentric") {
	LOG(GetName() + "::ctor");

	Requires(Robot::driveTrain);
}

// ==========================================================================

void FieldCentric::Initialize() {
	Robot::driveTrain->CrabInit();
}

// ==========================================================================

void FieldCentric::Execute() {
	auto x = Robot::oi->GetJoystickX();
	auto y = Robot::oi->GetJoystickY();
	auto z = Robot::oi->GetJoystickZ();
	Robot::driveTrain->FieldCentricCrab(z, -y, x, true);
}

// ==========================================================================

bool FieldCentric::IsFinished() {
	return false;
}

// ==========================================================================

void FieldCentric::End() {
}

// ==========================================================================

void FieldCentric::Interrupted() {
	End();
}

// ==========================================================================
