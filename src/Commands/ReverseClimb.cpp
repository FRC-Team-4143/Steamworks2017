#include "ReverseClimb.h"
#include "Robot.h"

ReverseClimb::ReverseClimb() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::climber);

}

// Called just before this Command runs the first time
void ReverseClimb::Initialize() {

	Robot::climber->reverseClimb();
}

// Called repeatedly when this Command is scheduled to run
void ReverseClimb::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool ReverseClimb::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReverseClimb::End() {

	Robot::climber->stopClimb();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReverseClimb::Interrupted() {
	End();
}
