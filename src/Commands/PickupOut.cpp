#include "PickupOut.h"
#include "Robot.h"

PickupOut::PickupOut() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::pickup);
}

void PickupOut::Initialize() {
	Robot::pickup->Out();
}

void PickupOut::Execute() {
}

bool PickupOut::IsFinished() {
	return false;
}

void PickupOut::End() {
	Robot::pickup->Stop();
}

void PickupOut::Interrupted() {
	End();
}
