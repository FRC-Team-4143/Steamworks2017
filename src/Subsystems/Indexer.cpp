#include "Indexer.h"
#include "../RobotMap.h"

Indexer::Indexer() : Subsystem("Indexer") {

	indexMotor = RobotMap::indexMotor;

	indexMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	indexMotor->SetControlMode(CANSpeedController::kPosition);
	indexMotor->ConfigNominalOutputVoltage(0.0, 0.0);
	indexMotor->ConfigPeakOutputVoltage(12.0, -12.0);
	indexMotor->SetSensorDirection(true);
	indexMotor->SetP(0.8);
	indexMotor->SetI(0.0);
	indexMotor->SetD(0.05);
	indexMotor->SetF(0.0);
}

void Indexer::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Indexer::SpinCW() {

	indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
	indexMotor->Set(0.7);
}

void Indexer::SpinCCW() {

	indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
	indexMotor->Set(-0.7);
}

void Indexer::SpinBall() {

	indexMotor->SetControlMode(CANSpeedController::kPosition);
	indexMotor->SetSetpoint(indexMotor->GetEncPosition()+4096);
}
void Indexer::Stop() {
	indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
	indexMotor->Set(0);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
