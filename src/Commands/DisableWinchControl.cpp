#include "Commands/DisableWinchControl.h"
#include "Robot.h"

// ==========================================================================

DisableWinchControl::DisableWinchControl() {
}

// ==========================================================================

void DisableWinchControl::Initialize() {
	Robot::winchSub->DisablePositionControl();
}

// ==========================================================================

void DisableWinchControl::Execute() {
}

// ==========================================================================

bool DisableWinchControl::IsFinished() {
	return true;
}

// ==========================================================================

void DisableWinchControl::End() {
}

// ==========================================================================

void DisableWinchControl::Interrupted() {
	End();
}

// ==========================================================================
