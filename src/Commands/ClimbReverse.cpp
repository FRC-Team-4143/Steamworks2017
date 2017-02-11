#include "Commands/ClimbReverse.h"
#include "Robot.h"

// ==========================================================================

ClimbReverse::ClimbReverse() {
	Requires(Robot::winchSub);
}

// ==========================================================================

void ClimbReverse::Initialize() {
}

// ==========================================================================

void ClimbReverse::Execute() {
	Robot::winchSub->ClimbReverse();
}

// ==========================================================================

bool ClimbReverse::IsFinished() {
	return false;
}

// ==========================================================================

void ClimbReverse::End() {
	Robot::winchSub->StopClimb();
}

// ==========================================================================

void ClimbReverse::Interrupted() {
	End();
}

// ==========================================================================
