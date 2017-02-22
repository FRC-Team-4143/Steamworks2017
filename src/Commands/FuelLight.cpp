#include "FuelLight.h"
#include "Robot.h"

FuelLight::FuelLight() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

void FuelLight::Initialize() {

}

void FuelLight::Execute() {
	RobotMap::i2c->Write(3, 0);
}


bool FuelLight::IsFinished() {
	return false;
}

void FuelLight::End() {
	RobotMap::i2c->Write(6, 0);
}

void FuelLight::Interrupted() {
	End();
}
