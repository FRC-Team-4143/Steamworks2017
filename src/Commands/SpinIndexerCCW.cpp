#include "SpinIndexerCCW.h"
#include "Robot.h"


SpinIndexerCCW::SpinIndexerCCW() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::indexer);

}

// Called just before this Command runs the first time
void SpinIndexerCCW::Initialize() {



	RobotMap::i2c->Write(9, 0);
}

// Called repeatedly when this Command is scheduled to run
void SpinIndexerCCW::Execute() {
	Robot::indexer->SpinCCW();
}

// Make this return true when this Command no longer needs to run execute()
bool SpinIndexerCCW::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void SpinIndexerCCW::End() {

	Robot::indexer->Stop();

	RobotMap::i2c->Write(6, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SpinIndexerCCW::Interrupted() {

	End();
}
