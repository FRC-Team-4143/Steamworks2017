#include "CANTalon.h"
#include "Shooter.h"
#include "../RobotMap.h"
#include "../Commands/ShootDefault.h"

//const int RPM = 7000;


Shooter::Shooter() : Subsystem("Shooter") {

   shooterLeftFront = RobotMap::shooterLeftFront;
   shooterLeftBack = RobotMap::shooterLeftBack;
   shooterRightFront = RobotMap::shooterRightFront;
   shooterRightBack = RobotMap::shooterRightBack;

   shooterLeftFront->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
   shooterRightFront->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);

   shooterLeftFront->SetControlMode(CANSpeedController::kSpeed);
   shooterLeftFront->ConfigNominalOutputVoltage(0.0, 0.0);
   shooterLeftFront->ConfigPeakOutputVoltage(12.0, -12.0);
   shooterLeftFront->SetSensorDirection(true);
   shooterLeftFront->SetP(0.01);
   shooterLeftFront->SetI(0.0);
   shooterLeftFront->SetD(0.0);
   shooterLeftFront->SetF(0.012);


   shooterRightFront->SetControlMode(CANSpeedController::kSpeed);
   shooterRightFront->ConfigNominalOutputVoltage(0.0, 0.0);
   shooterRightFront->ConfigPeakOutputVoltage(12.0, -12.0);
   shooterRightFront->SetSensorDirection(true);
   shooterRightFront->SetP(0.01);
   shooterRightFront->SetI(0.0);
   shooterRightFront->SetD(0.0);
   shooterRightFront->SetF(0.012);

   SmartDashboard::PutNumber("Set Shooter Speed", 7000);

}

void Shooter::InitDefaultCommand() { SetDefaultCommand(new ShootDefault()); }

void Shooter::shootFront() {

	double RPM = SmartDashboard::GetNumber("Set Shooter Speed", 7000);
	//shooterLeftFront->SetControlMode(CANSpeedController::kPercentVbus);
	//shooterLeftFront->Set(0.75);
	//shooterRightFront->SetControlMode(CANSpeedController::kPercentVbus);
	//shooterRightFront->Set(-0.75);
	shooterLeftFront->SetControlMode(CANSpeedController::kSpeed);
	shooterLeftFront->Set(RPM);
	shooterRightFront->SetControlMode(CANSpeedController::kSpeed);
	shooterRightFront->Set(-RPM);

}

void Shooter::shootBack() {

	//shooterLeftBack->Set(-1);
	//shooterRightBack->Set(1);
}

void Shooter::stopFront() {
	shooterLeftFront->SetControlMode(CANSpeedController::kPercentVbus);
	shooterLeftFront->Set(0);
	shooterRightFront->SetControlMode(CANSpeedController::kPercentVbus);
	shooterRightFront->Set(0);
}

void Shooter::stopBack() {
	shooterLeftBack->Set(0);
	shooterRightBack->Set(0);
}

void Shooter::shootDefault(float right, float left) {
	SmartDashboard::PutNumber("Bottom Velocity", getVelocity());
	SmartDashboard::PutNumber("Top Velocity", shooterRightFront->GetSpeed());
	if(right > 0.1) {
		shootFront();
		shootBack();
  } else {
		stopFront();
		stopBack();
  }
}

double Shooter::getVelocity() { return shooterLeftFront->GetSpeed(); }


void Shooter::readValues(){
}
