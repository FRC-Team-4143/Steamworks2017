#include "Climber.h"
#include "../RobotMap.h"

Climber::Climber() : Subsystem("Climber") {
	climbingMotor = RobotMap::climbingMotor;
	climbingMotor2 = RobotMap::climbingMotor2;
}

void Climber::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Climber::climb(float speed) {
	climbingMotor->SetControlMode(CANSpeedController::kPercentVbus);
	climbingMotor->Set(-speed);

	climbingMotor2->SetControlMode(CANSpeedController::kPercentVbus);
	climbingMotor2->Set(speed);
}

void Climber::reverseClimb(float speed) {
	climbingMotor->SetControlMode(CANSpeedController::kPercentVbus);
	climbingMotor->Set(speed);
}

void Climber::stopClimb() {
	climbingMotor->Set(0);
	climbingMotor2->Set(0);

}
