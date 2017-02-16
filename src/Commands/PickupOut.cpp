#include "PickupOut.h"
#include "Robot.h"

PickupOut::PickupOut() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::pickup);

}

// Called just before this Command runs the first time
void PickupOut::Initialize() {

	Robot::pickup->Out();

}

// Called repeatedly when this Command is scheduled to run
void PickupOut::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool PickupOut::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void PickupOut::End() {

	Robot::pickup->Stop();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PickupOut::Interrupted() {

	End();
}
