#include "SpinIndexerCCW.h"
#include "Robot.h"

SpinIndexerCCW::SpinIndexerCCW() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::indexer);
}

void SpinIndexerCCW::Initialize() {
	RobotMap::i2c->Write(11, 0);
	//RobotMap::i2c->Write(9, 0);
}

void SpinIndexerCCW::Execute() {
	Robot::indexer->SpinCCW();
}

bool SpinIndexerCCW::IsFinished() {
	return false;
}

void SpinIndexerCCW::End() {
	Robot::indexer->Stop();
}

void SpinIndexerCCW::Interrupted() {
	End();
}
