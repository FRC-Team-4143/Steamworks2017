#include "Commands/Feed.h"
#include "Robot.h"

// ==========================================================================

Feed::Feed(double seconds)
: _seconds(seconds), _seenBall(false){
	Requires(Robot::shooter);
}

// ==========================================================================

void Feed::Initialize() {
	SetTimeout(_seconds);
	_seenBall = false;
	Robot::shooter->ShootFront();
	Robot::shooter->ShootBack();
}

// ==========================================================================

void Feed::Execute() {
	if (Robot::shooter->IsAtShootingSpeed()) {
		Robot::shooter->Feed();
	}

	if (Robot::shooter->SeesBall()) {
		_seenBall = true;
	}
}

// ==========================================================================

bool Feed::IsFinished() {
	return IsTimedOut() || (_seenBall && !Robot::shooter->SeesBall());
}

// ==========================================================================

void Feed::End() {
	Robot::shooter->StopFeed();
}

// ==========================================================================

void Feed::Interrupted() {
	End();
}

// ==========================================================================
