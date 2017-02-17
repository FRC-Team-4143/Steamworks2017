#include "OneBall.h"
#include "Robot.h"

OneBall::OneBall() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::indexer);

}

// Called just before this Command runs the first time
void OneBall::Initialize() {

	Robot::indexer->SpinBall();
}

// Called repeatedly when this Command is scheduled to run
void OneBall::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool OneBall::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void OneBall::End() {
	Robot::indexer->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void OneBall::Interrupted() {

}
