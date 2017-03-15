#include "CalculateSpeed.h"
#include "Robot.h"

CalculateSpeed::CalculateSpeed() {
	// Use Requires() here to declare subsystem dependencies
	SetRunWhenDisabled(true);
}

void CalculateSpeed::Initialize() {
}

void CalculateSpeed::Execute() {

}

bool CalculateSpeed::IsFinished() {
	Robot::shooter->calculateSpeed();
	return true;
}

void CalculateSpeed::End() {
	Robot::climber->stopClimb();
}

void CalculateSpeed::Interrupted() {
	End();
}
