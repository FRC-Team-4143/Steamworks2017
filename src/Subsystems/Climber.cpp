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
	climbingMotor->Set(ControlMode::PercentOutput, -speed);

	//climbingMotor2->SetControlMode(CANSpeedController::kPercentVbus);
	//climbingMotor2->Set(speed);
}

void Climber::reverseClimb(float speed) {
	climbingMotor->Set(ControlMode::PercentOutput, speed);
}

void Climber::stopClimb() {
	climbingMotor->Set(ControlMode::PercentOutput, 0);
	climbingMotor2->Set(0);

}
