#include "SpinIndexerCW.h"
#include "Robot.h"

SpinIndexerCW::SpinIndexerCW() {
	Requires(Robot::indexer);
}

void SpinIndexerCW::Initialize() {
	RobotMap::i2c->Write(11, 0);
	//RobotMap::i2c->Write(5, 0);
}

void SpinIndexerCW::Execute() {
	Robot::indexer->SpinCW();
}

bool SpinIndexerCW::IsFinished() {
	return false;
}

void SpinIndexerCW::End() {
	Robot::indexer->Stop();
	RobotMap::i2c->Write(6, 0);
}

void SpinIndexerCW::Interrupted() {
	End();
}
