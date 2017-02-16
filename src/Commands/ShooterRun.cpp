#include <Commands/ShooterRun.h>
#include "Robot.h"

//const int SHOOTER_RPM = 4000;
//const int SHOOTER_RPM_TOLERANCE = 50;

// ==========================================================================

ShooterRun::ShooterRun() {
	//Requires(Robot::shooter.get());
}

// ==========================================================================

void ShooterRun::Initialize() {
	Robot::shooter->Start();
}

// ==========================================================================

void ShooterRun::Execute() {
}

// ==========================================================================

bool ShooterRun::IsFinished() {
	return false;
}

// ==========================================================================

void ShooterRun::End() {
	Robot::shooter->Stop();
}

// ==========================================================================

void ShooterRun::Interrupted() {
	End();
}

// ==========================================================================
