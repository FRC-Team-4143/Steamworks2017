#include "ScriptIndex.h"
#include "Robot.h"

ScriptIndex::ScriptIndex(float speed, float timeout) {
	Requires(Robot::indexer);
	_timeout = timeout;
	_speed = speed;
}

void ScriptIndex::Initialize() {
	SetTimeout(_timeout);
	RobotMap::i2c->Write(11, 0);
}

void ScriptIndex::Execute() {
	Robot::indexer->SpinCW(_speed);
}

bool ScriptIndex::IsFinished() {
	return IsTimedOut();
}

void ScriptIndex::End() {
	Robot::indexer->Stop();
	RobotMap::i2c->Write(6, 0);
}

void ScriptIndex::Interrupted() {
	End();
}
