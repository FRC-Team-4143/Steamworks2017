#include "GearLight.h"
#include "Robot.h"

GearLight::GearLight() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

void GearLight::Initialize() {

}

void GearLight::Execute() {
	RobotMap::i2c->Write(4, 0);
}

bool GearLight::IsFinished() {
	return false;
}

void GearLight::End() {
	RobotMap::i2c->Write(6, 0);
}

void GearLight::Interrupted() {
	End();
}
