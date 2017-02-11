#include "Commands/Shoot.h"
#include "Robot.h"

// ==========================================================================

Shoot::Shoot(double timeoutSeconds)
: _timeoutSeconds(timeoutSeconds) {
	Requires(Robot::shooter);
}

// ==========================================================================

void Shoot::Initialize() {
	SetTimeout(_timeoutSeconds);
	Robot::shooter->ShootFront();
	Robot::shooter->ShootBack();
}

// ==========================================================================

void Shoot::Execute() {
}

// ==========================================================================

bool Shoot::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void Shoot::End() {
}

// ==========================================================================

void Shoot::Interrupted() {
	End();
}

// ==========================================================================
