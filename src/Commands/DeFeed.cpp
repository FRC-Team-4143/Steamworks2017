#include "Commands/DeFeed.h"
#include "Robot.h"

// ==========================================================================

DeFeed::DeFeed(double timeoutSeconds)
: _timeoutSeconds(timeoutSeconds) {
	Requires(Robot::shooter);
}

// ==========================================================================

void DeFeed::Initialize() {
	SetTimeout(_timeoutSeconds);
	Robot::shooter->DeFeed();
}

// ==========================================================================

void DeFeed::Execute() {
}

// ==========================================================================

bool DeFeed::IsFinished() {
	return IsTimedOut() || Robot::shooter->SeesBall();
}

// ==========================================================================

void DeFeed::End() {
	Robot::shooter->StopFeed();
}

// ==========================================================================

void DeFeed::Interrupted() {
	End();
}

// ==========================================================================
