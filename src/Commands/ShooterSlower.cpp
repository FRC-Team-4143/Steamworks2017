#include <Commands/ShooterSlower.h>
#include "Robot.h"

// ==========================================================================

ShooterSlower::ShooterSlower() {
	// Do NOT require the Shooter subsystem. Doing so will interrupt the Run command.
	//Requires(Robot::shooter.get());
}

// ==========================================================================

void ShooterSlower::Initialize() {
	Robot::shooter->DecreaseTargetRpm();
}

// ==========================================================================

void ShooterSlower::Execute() {
}

// ==========================================================================

bool ShooterSlower::IsFinished() {
	return true;
}

// ==========================================================================

void ShooterSlower::End() {
}

// ==========================================================================

void ShooterSlower::Interrupted() {
	End();
}

// ==========================================================================
