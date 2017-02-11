#include "Commands/DefeedShoot.h"
#include "Robot.h"

// ==========================================================================

DefeedShoot::DefeedShoot(double timeoutSeconds)
: _timeoutSeconds(timeoutSeconds) {
	Requires(Robot::shooter);
}

// ==========================================================================

void DefeedShoot::Initialize() {
	SetTimeout(_timeoutSeconds);
	Robot::shooter->DeFeed();
}

// ==========================================================================

void DefeedShoot::Execute() {
	if (Robot::shooter->SeesBall()) {
		Robot::shooter->ShootFront();
		Robot::shooter->ShootBack();
		Robot::shooter->StopFeed();
	}

	if (Robot::oi->GetLeftTrigger() > 0.1) {
		Robot::shooter->Feed();
	}
}

// ==========================================================================

bool DefeedShoot::IsFinished() {
	return IsTimedOut() || (Robot::shooter->SeesBall() && Robot::shooter->IsAtShootingSpeed());
}

// ==========================================================================

void DefeedShoot::End() {
	Robot::shooter->StopFeed();
}

// ==========================================================================

void DefeedShoot::Interrupted() {
	End();
}

// ==========================================================================
