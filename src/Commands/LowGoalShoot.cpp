#include "Commands/LowGoalShoot.h"
#include "Robot.h"

// ==========================================================================

LowGoalShoot::LowGoalShoot(double timeoutSeconds)
: _counter(0), _timeoutSeconds(timeoutSeconds) {
	Requires(Robot::shooter);
}

// ==========================================================================

void LowGoalShoot::Initialize() {
	_counter = 0;
	SetTimeout(_timeoutSeconds);
	Robot::shooter->ShootFront();
}

// ==========================================================================

void LowGoalShoot::Execute() {
	_counter++;
	if (_counter > 10) {
		Robot::shooter->ShootBack();
	}
}

// ==========================================================================

bool LowGoalShoot::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void LowGoalShoot::End() {
}

// ==========================================================================

void LowGoalShoot::Interrupted() {
	End();
}

// ==========================================================================
