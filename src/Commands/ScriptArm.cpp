#include "Commands/ScriptArm.h"
#include "Robot.h"

// ==========================================================================

ScriptArm::ScriptArm(int pos)
: _pos(pos) {
	Requires(Robot::armSub);
}

// ==========================================================================

void ScriptArm::Initialize() {
	SetTimeout(0.5);
	if (_pos == 0) {
		Robot::armSub->StowArm();
	}
	else if (_pos == 1) {
		Robot::armSub->ArmUp();
	}
	else if (_pos == 2) {
		Robot::armSub->ArmDown();
	}
}

// ==========================================================================

void ScriptArm::Execute() {
}

// ==========================================================================

bool ScriptArm::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptArm::End() {
	Robot::armSub->DisablePositionControl();
}

// ==========================================================================

void ScriptArm::Interrupted() {
	End();
}

// ==========================================================================
