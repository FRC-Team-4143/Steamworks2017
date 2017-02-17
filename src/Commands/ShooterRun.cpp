#include <Commands/ShooterRun.h>
#include "Robot.h"

//const int SHOOTER_RPM = 4000;
//const int SHOOTER_RPM_TOLERANCE = 50;

// ==========================================================================

ShooterRun::ShooterRun() {
	Requires(Robot::shooter);
}

// ==========================================================================

void ShooterRun::Initialize() {

}

// ==========================================================================

void ShooterRun::Execute() {
	Robot::shooter->shootFront();
}

// ==========================================================================

bool ShooterRun::IsFinished() {
	return false;
}

// ==========================================================================

void ShooterRun::End() {
	Robot::shooter->stopFront();
}

// ==========================================================================

void ShooterRun::Interrupted() {
	End();
}

// ==========================================================================
