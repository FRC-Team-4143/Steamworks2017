#include "Turret.h"
#include "RobotMap.h"
#include "Commands/DefaultTurret.h"
#include "Robot.h"

Turret::Turret() : Subsystem("Turret") {
	turretMotor = RobotMap::turretMotor;
	//indexMotor = RobotMap::indexMotor;
}

void Turret::InitDefaultCommand() {
	SetDefaultCommand(new DefaultTurret());
}

void Turret::SetSpeed(double speed) {
	SmartDashboard::PutNumber("Turret Rotation", turretMotor->GetPosition());

	if (speed > 0) {
		if (turretMotor->GetPosition() > 3) {
			speed = 0;
		}
	}
	else if (speed < 0) {
		if (turretMotor->GetPosition() < -3) {
			speed = 0;
		}
	}

	turretMotor->SetControlMode(CANTalon::kSpeed);
	turretMotor->Set(speed);
	Robot::indexer->SetSpeed(speed);
}

void Turret::Stop() {
	turretMotor->Set(0);
	Robot::indexer->SetSpeed(0);
}
