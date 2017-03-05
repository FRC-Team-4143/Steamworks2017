#include "Climber.h"
#include "../RobotMap.h"

Climber::Climber() : Subsystem("Climber") {
	climbingMotor = RobotMap::climbingMotor;
}

void Climber::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Climber::climb(float speed) {
	climbingMotor->SetControlMode(CANSpeedController::kPercentVbus);
	climbingMotor->Set(-speed);
}

void Climber::reverseClimb(float speed) {
	climbingMotor->SetControlMode(CANSpeedController::kPercentVbus);
	climbingMotor->Set(speed);
}

void Climber::stopClimb() {
	climbingMotor->Set(0);
}
