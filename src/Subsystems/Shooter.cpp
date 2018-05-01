#include "ctre/phoenix.h"
#include "Shooter.h"
#include "../RobotMap.h"
#include "../Robot.h"
#include "../Commands/BoilerLineup.h"

//const int RPM = 7000;

Shooter::Shooter() : Subsystem("Shooter") {
   shooterMotor1 = RobotMap::shooterMotor1;
   shooterMotor2 = RobotMap::shooterMotor2;

   shooterMotor1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
   shooterMotor2->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0 ,10);
/*
   shooterMotor1->ControlMode(CANSpeedController::kSpeed);
   shooterMotor1->ConfigNominalOutputVoltage(0.0, 0.0);
   shooterMotor1->ConfigPeakOutputVoltage(12.0, -12.0);
   shooterMotor1->SetSensorDirection(true);
   shooterMotor1->Config_kP(0, 0.04, 0);
   shooterMotor1->Config_kI(0, 0.0, 0);
   shooterMotor1->Config_kD(0, 0, 0);
   shooterMotor1->Config_kF(0, 0.028, 0);

   shooterMotor2->SetControlMode(CANSpeedController::kSpeed);
   shooterMotor2->ConfigNominalOutputVoltage(0.0, 0.0);
   shooterMotor2->ConfigPeakOutputVoltage(12.0, -12.0);
   shooterMotor2->SetSensorDirection(true);
   shooterMotor2->SetP(0.04);
   shooterMotor2->SetI(0.0);
   shooterMotor2->SetD(0.0);
   shooterMotor2->SetF(0.02827);
*/
   //SmartDashboard::PutNumber("Set Shooter Speed", 7000);
   targetShooterSpeed = 3000;
}

void Shooter::InitDefaultCommand() {
	//SetDefaultCommand(new BoilerLineup());
}

void Shooter::shootFront() {
	//shooterMotor1->SetControlMode(CANSpeedController::kPercentVbus);
	//shooterMotor1->Set(.35);
	//shooterMotor2->SetControlMode(CANSpeedController::kPercentVbus);
	//shooterMotor2->Set(-.35);
	shooterMotor1->Set(ControlMode::Velocity, targetShooterSpeed);
	//shooterMotor1->Set(targetShooterSpeed);
	shooterMotor2->Set(ControlMode::Velocity, targetShooterSpeed);
	//shooterMotor2->Set(targetShooterSpeed);
}

void Shooter::shootBack() {
	//shooterLeftBack->Set(-1);
	//shooterRightBack->Set(1);
}

void Shooter::stopFront() {
	shooterMotor1->Set(ControlMode::PercentOutput, 0);
	shooterMotor1->Set(0);
	shooterMotor2->Set(ControlMode::PercentOutput, 0);
	shooterMotor2->Set(0);
}

void Shooter::stopBack() {
	shooterMotor1->Set(0);
	shooterMotor2->Set(0);
}

void Shooter::shootDefault(float right, float left) {
	SmartDashboard::PutNumber("Bottom Velocity", shooterMotor1->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("Top Velocity", shooterMotor2->GetSelectedSensorVelocity(0));
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
	return shooterMotor1->GetSelectedSensorVelocity(0);
}

void Shooter::readValues() {
}

//-128
//2700 -113
//-98
//2800 -83
//-67
//2900 -51
//-39
//3000 -27
//-15
//3100 -4
//8
//3200 19
//29

int Shooter::calculateSpeed(){
	int distance = Robot::visionBridge->GetBoilerDistance();
	if (distance > 29)
		targetShooterSpeed = 3300;
	else if (distance < 29 && distance > 8)
		targetShooterSpeed = 3200;
	else if (distance < 8 && distance > -15)
		targetShooterSpeed = 3100;
	else if (distance < -15 && distance > -39)
		targetShooterSpeed = 3000;
	else if (distance < -39 && distance > -67)
		targetShooterSpeed = 2900;
	else if (distance < -67 && distance > -98)
		targetShooterSpeed = 2800;
	else if (distance < -98 && distance > -128)
		targetShooterSpeed = 2700;
	else if (distance < -128)
		targetShooterSpeed = 2600;

	return targetShooterSpeed;
}
