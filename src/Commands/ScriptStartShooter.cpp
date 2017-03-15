#include <Commands/ScriptStartShooter.h>
#include "Robot.h"

// ==========================================================================

ScriptStartShooter::ScriptStartShooter(int speed) {
	Requires(Robot::shooter);
	_speed = speed;
}

// ==========================================================================

ScriptStartShooter::ScriptStartShooter() {
	Requires(Robot::shooter);
	_speed = Robot::shooter->calculateSpeed();
}

// ==========================================================================

void ScriptStartShooter::Initialize() {
}

// ==========================================================================

void ScriptStartShooter::Execute() {
}

// ==========================================================================

bool ScriptStartShooter::IsFinished() {
	Robot::shooter->targetShooterSpeed = _speed;
	Robot::shooter->shootFront();
	RobotMap::i2c->Write(0, 0);
	return true;
}

// ==========================================================================

void ScriptStartShooter::End() {

}

// ==========================================================================

void ScriptStartShooter::Interrupted() {
	End();
}

// ==========================================================================
