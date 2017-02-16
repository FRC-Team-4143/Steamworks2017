#include <Commands/ShooterFaster.h>
#include "Robot.h"

// ==========================================================================

ShooterFaster::ShooterFaster() {
	// Do NOT require the Shooter subsystem. Doing so will interrupt the Run command.
	//Requires(Robot::shooter.get());
}

// ==========================================================================

void ShooterFaster::Initialize() {
	Robot::shooter->IncreaseTargetRpm();
}

// ==========================================================================

void ShooterFaster::Execute() {
}

// ==========================================================================

bool ShooterFaster::IsFinished() {
	return true;
}

// ==========================================================================

void ShooterFaster::End() {
}

// ==========================================================================

void ShooterFaster::Interrupted() {
	End();
}

// ==========================================================================
