#include "CANTalon.h"
#include "Shooter.h"
#include "../RobotMap.h"

//const int RPM = 7000;

Shooter::Shooter() : Subsystem("Shooter") {
   shooterMotor1 = RobotMap::shooterMotor1;
   shooterMotor2 = RobotMap::shooterMotor2;

   shooterMotor1->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
   shooterMotor2->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);

   shooterMotor1->SetControlMode(CANSpeedController::kSpeed);
   shooterMotor1->ConfigNominalOutputVoltage(0.0, 0.0);
   shooterMotor1->ConfigPeakOutputVoltage(12.0, -12.0);
   shooterMotor1->SetSensorDirection(true);
   shooterMotor1->SetP(0.08);
   shooterMotor1->SetI(0.0);
   shooterMotor1->SetD(0);
   shooterMotor1->SetF(0.02827);

   shooterMotor2->SetControlMode(CANSpeedController::kSpeed);
   shooterMotor2->ConfigNominalOutputVoltage(0.0, 0.0);
   shooterMotor2->ConfigPeakOutputVoltage(12.0, -12.0);
   shooterMotor2->SetSensorDirection(true);
   shooterMotor2->SetP(0.02);
   shooterMotor2->SetI(0.0);
   shooterMotor2->SetD(0.0);
   shooterMotor2->SetF(0.02827);

   //SmartDashboard::PutNumber("Set Shooter Speed", 7000);
   targetShooterSpeed = 0;
}

void Shooter::InitDefaultCommand() {
	//SetDefaultCommand(new ShootDefault());
}

void Shooter::shootFront() {
	//shooterMotor1->SetControlMode(CANSpeedController::kPercentVbus);
	//shooterMotor1->Set(.35);
	//shooterMotor2->SetControlMode(CANSpeedController::kPercentVbus);
	//shooterMotor2->Set(-.35);
	shooterMotor1->SetControlMode(CANSpeedController::kSpeed);
	shooterMotor1->Set(targetShooterSpeed);
	shooterMotor2->SetControlMode(CANSpeedController::kSpeed);
	shooterMotor2->Set(targetShooterSpeed);
}

void Shooter::shootBack() {
	//shooterLeftBack->Set(-1);
	//shooterRightBack->Set(1);
}

void Shooter::stopFront() {
	shooterMotor1->SetControlMode(CANSpeedController::kPercentVbus);
	shooterMotor1->Set(0);
	shooterMotor2->SetControlMode(CANSpeedController::kPercentVbus);
	shooterMotor2->Set(0);
}

void Shooter::stopBack() {
	shooterMotor1->Set(0);
	shooterMotor2->Set(0);
}

void Shooter::shootDefault(float right, float left) {
	SmartDashboard::PutNumber("Bottom Velocity", shooterMotor1->GetSpeed());
	SmartDashboard::PutNumber("Top Velocity", shooterMotor2->GetSpeed());
	SmartDashboard::PutNumber("Target Shooter Speed", targetShooterSpeed);
	if(right > 0.1) {
		shootFront();
		shootBack();
	}
	else {
		stopFront();
		stopBack();
	}
}

void Shooter::IncreaseTargetRpm() {
	targetShooterSpeed += 100;
}

void Shooter::DecreaseTargetRpm() {
	if (targetShooterSpeed - 100 >= 0){
		targetShooterSpeed -= 100;
	} else {
		targetShooterSpeed = 0;
	}
}

double Shooter::getVelocity() {
	return shooterMotor1->GetSpeed();
}

void Shooter::readValues() {
}
