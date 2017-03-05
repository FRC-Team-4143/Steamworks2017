#include "ReverseClimb.h"
#include "Robot.h"

ReverseClimb::ReverseClimb() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::climber);
}

void ReverseClimb::Initialize() {
}

void ReverseClimb::Execute() {
	if (Robot::oi->GetRightTrigger() > 0.75)
		Robot::climber->reverseClimb(1);
	else {
		Robot::climber->reverseClimb(0.5);
	}
}

bool ReverseClimb::IsFinished() {
	return false;
}

void ReverseClimb::End() {
	Robot::climber->stopClimb();
}

void ReverseClimb::Interrupted() {
	End();
}
