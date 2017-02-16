#include "Indexer.h"
#include "../RobotMap.h"

Indexer::Indexer() : Subsystem("Indexer") {

	indexMotor = RobotMap::indexMotor;
}

void Indexer::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Indexer::SpinCW() {

	indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
	indexMotor->Set(0.25);
}

void Indexer::SpinCCW() {

	indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
	indexMotor->Set(-0.25);
}

void Indexer::Stop() {

	indexMotor->Set(0);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
