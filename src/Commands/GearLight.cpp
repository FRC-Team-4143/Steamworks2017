#include "GearLight.h"
#include "Robot.h"

GearLight::GearLight() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void GearLight::Initialize() {
	RobotMap::i2c->Write(4, 0);
}

// Called repeatedly when this Command is scheduled to run
void GearLight::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool GearLight::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void GearLight::End() {
	RobotMap::i2c->Write(6, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GearLight::Interrupted() {
	RobotMap::i2c->Write(6, 0);
}
