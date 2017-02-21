#include "FuelLight.h"
#include "Robot.h"

FuelLight::FuelLight() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void FuelLight::Initialize() {
	RobotMap::i2c->Write(3, 0);
}

// Called repeatedly when this Command is scheduled to run
void FuelLight::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool FuelLight::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void FuelLight::End() {
	RobotMap::i2c->Write(6, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FuelLight::Interrupted() {
	RobotMap::i2c->Write(6, 0);
}
