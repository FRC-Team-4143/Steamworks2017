#include "Climb.h"
#include "Robot.h"

Climb::Climb() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::climber);
}

void Climb::Initialize() {
}

void Climb::Execute() {
	if (Robot::oi->GetRightTrigger() > 0.75)
		Robot::climber->climb(1);
	else {
		Robot::climber->climb(0.5);
	}
}

bool Climb::IsFinished() {
	return false;
}

void Climb::End() {
	Robot::climber->stopClimb();
}

void Climb::Interrupted() {
	End();
}
