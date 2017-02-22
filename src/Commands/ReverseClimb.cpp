#include "ReverseClimb.h"
#include "Robot.h"

ReverseClimb::ReverseClimb() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::climber);
}

void ReverseClimb::Initialize() {
	Robot::climber->reverseClimb();
}

void ReverseClimb::Execute() {
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
