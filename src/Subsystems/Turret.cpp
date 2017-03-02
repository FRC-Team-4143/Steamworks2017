#include "Turret.h"
#include "RobotMap.h"
#include "Commands/DefaultTurret.h"
#include "Robot.h"

Turret::Turret() : Subsystem("Turret") {
	turretMotor = RobotMap::turretMotor;

		SmartDashboard::PutNumber("Turret Max Speed",0);
}

void Turret::InitDefaultCommand() {
	SetDefaultCommand(new DefaultTurret());
}

void Turret::SetSpeed(double speed) {
	//SmartDashboard::PutNumber("Turret Rotation", turretMotor->GetPosition());
/*
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
*/
	turretMotor->SetControlMode(CANTalon::kPercentVbus);
	turretMotor->Set(speed);

}

void Turret::Stop() {
	turretMotor->Set(0);
	Robot::indexer->SetSpeed(0);
}
