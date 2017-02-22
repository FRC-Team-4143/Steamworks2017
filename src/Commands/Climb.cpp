#include "Climb.h"
#include "Robot.h"

Climb::Climb() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::climber);
}

void Climb::Initialize() {
	Robot::climber->climb();
}

void Climb::Execute() {
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
