#include "PickupIn.h"
#include "Robot.h"

PickupIn::PickupIn() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::pickup);
}

void PickupIn::Initialize() {
	Robot::pickup->In();
}

void PickupIn::Execute() {
}

bool PickupIn::IsFinished() {
	return false;
}

void PickupIn::End() {
	Robot::pickup->Stop();
}

void PickupIn::Interrupted() {
	End();
}
