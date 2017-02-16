#include <Commands/ShooterDefault.h>
#include "Robot.h"

// ==========================================================================

ShooterDefault::ShooterDefault() {
	Requires(Robot::shooter.get());
}

// ==========================================================================

void ShooterDefault::Initialize() {
}

// ==========================================================================

void ShooterDefault::Execute() {
	Robot::shooter->Report();
}

// ==========================================================================

bool ShooterDefault::IsFinished() {
	return false;
}

// ==========================================================================

void ShooterDefault::End() {
}

// ==========================================================================

void ShooterDefault::Interrupted() {
	End();
}

// ==========================================================================
