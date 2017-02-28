#include <Commands/ScriptStopShooter.h>
#include "Robot.h"

// ==========================================================================

ScriptStopShooter::ScriptStopShooter() {
	Requires(Robot::shooter);
}

// ==========================================================================

void ScriptStopShooter::Initialize() {
}

// ==========================================================================

void ScriptStopShooter::Execute() {
}

// ==========================================================================

bool ScriptStopShooter::IsFinished() {
	Robot::shooter->stopFront();
	RobotMap::i2c->Write(6, 0);
	return true;
}

// ==========================================================================

void ScriptStopShooter::End() {

}

// ==========================================================================

void ScriptStopShooter::Interrupted() {
	End();
}

// ==========================================================================
