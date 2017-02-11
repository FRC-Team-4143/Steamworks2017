#include "Commands/ShootDefault.h"
#include "Robot.h"

// ==========================================================================

ShootDefault::ShootDefault() {
	Requires(Robot::shooter);
}

// ==========================================================================

void ShootDefault::Initialize() {
}

// ==========================================================================

void ShootDefault::Execute() {
	Robot::shooter->Shoot(Robot::oi->GetRightTrigger(), Robot::oi->GetLeftTrigger());
}

// ==========================================================================

bool ShootDefault::IsFinished() {
	return false;
}

// ==========================================================================

void ShootDefault::End() {
	Robot::shooter->StopFront();
	Robot::shooter->StopBack();
}

// ==========================================================================

void ShootDefault::Interrupted() {
	End();
}

// ==========================================================================
