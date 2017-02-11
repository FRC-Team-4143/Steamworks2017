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
	Robot::shooter->shootDefault(Robot::oi->GetRightTrigger(),
								 Robot::oi->GetLeftTrigger());
}

// ==========================================================================

bool ShootDefault::IsFinished() {
	return false;
}

// ==========================================================================

void ShootDefault::End() {


}

// ==========================================================================

void ShootDefault::Interrupted() {
	End();
}

// ==========================================================================
