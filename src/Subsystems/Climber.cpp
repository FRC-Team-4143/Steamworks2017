#include "Climber.h"
#include "../RobotMap.h"

Climber::Climber() : Subsystem("Climber") {
	climbingMotor = RobotMap::climbingMotor;
}

void Climber::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Climber::climb() {
	climbingMotor->SetControlMode(CANSpeedController::kPercentVbus);
	climbingMotor->Set(1);
}

void Climber::reverseClimb() {
	climbingMotor->SetControlMode(CANSpeedController::kPercentVbus);
	climbingMotor->Set(-1);
}

void Climber::stopClimb() {
	climbingMotor->Set(0);
}
