#include "SpinIndexerCW.h"
#include "Robot.h"

SpinIndexerCW::SpinIndexerCW() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::indexer);
}

// Called just before this Command runs the first time
void SpinIndexerCW::Initialize() {

	Robot::indexer->SpinCW();

	RobotMap::i2c->Write(5, 0);
}

// Called repeatedly when this Command is scheduled to run
void SpinIndexerCW::Execute() {


}

// Make this return true when this Command no longer needs to run execute()
bool SpinIndexerCW::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void SpinIndexerCW::End() {

	Robot::indexer->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SpinIndexerCW::Interrupted() {

	End();
}
