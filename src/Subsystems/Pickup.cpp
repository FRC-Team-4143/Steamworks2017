#include "Pickup.h"
#include "RobotMap.h"

Pickup::Pickup() : Subsystem("Pickup") {
	pickup = RobotMap::pickup;
}

void Pickup::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Pickup::In() {
	pickup->Set(ControlMode::PercentOutput, 0.25);
	//pickup->Set(0.25);
}

void Pickup::Out() {
	pickup->Set(ControlMode::PercentOutput, -0.25);
	//pickup->Set(-0.25);
}

void Pickup::Stop() {
	pickup->Set(0);
}
