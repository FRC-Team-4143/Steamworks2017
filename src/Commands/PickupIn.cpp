#include "PickupIn.h"
#include "Robot.h"

PickupIn::PickupIn() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::pickup);

}

// Called just before this Command runs the first time
void PickupIn::Initialize() {

	Robot::pickup->In();
}

// Called repeatedly when this Command is scheduled to run
void PickupIn::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool PickupIn::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void PickupIn::End() {

	Robot::pickup->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PickupIn::Interrupted() {

	End();
}
