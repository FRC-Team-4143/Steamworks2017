#include "Commands/ResetWinch.h"
#include "Robot.h"

// ==========================================================================

ResetWinch::ResetWinch() {
	Requires(Robot::winchSub);
	SetRunWhenDisabled(true);
	SetTimeout(1);
}

// ==========================================================================

void ResetWinch::Initialize() {
	Robot::winchSub->Reset();
}

// ==========================================================================

void ResetWinch::Execute() {
}

// ==========================================================================

bool ResetWinch::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ResetWinch::End() {
}

// ==========================================================================

void ResetWinch::Interrupted() {
	End();
}

// ==========================================================================
