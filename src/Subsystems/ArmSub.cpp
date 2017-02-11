#include "Subsystems/ArmSub.h"
#include <SmartDashboard/SmartDashboard.h>
#include "RobotMap.h"

// ==========================================================================

ArmSub::ArmSub()
: frc::Subsystem("Arm"), _armMotor(nullptr), _offset(0) {
	_armMotor = RobotMap::armMotor;
	_armMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	_armMotor->SetP(0.8);
	_armMotor->SetI(0.005);
	_armMotor->SetD(0.05);
	_armMotor->SetF(0.0);
	//_armMotor->SetSensorDirection(true);
	_armMotor->SetSetpoint(0.0);
	_armMotor->ConfigPeakOutputVoltage(10.0, -10.0);
	_armMotor->SetControlMode(CANSpeedController::kPercentVbus);
	_armMotor->Set(0);
}

// ==========================================================================

void ArmSub::InitDefaultCommand() {
	//SetDefaultCommand(new MySpecialCommand());
}

// ==========================================================================

double ArmSub::GetRawPosition() const {
	return _armMotor->GetPosition();
}

// ==========================================================================

void ArmSub::ArmDown() {
	_armMotor->SetControlMode(CANSpeedController::kPosition);
	_armMotor->SetSetpoint(0.67 + _offset);
}

// ==========================================================================

void ArmSub::ArmUp() {
	_armMotor->SetControlMode(CANSpeedController::kPosition);
	_armMotor->SetSetpoint(0.43 + _offset);
}

// ==========================================================================

void ArmSub::DisablePositionControl() {
	_armMotor->SetControlMode(CANSpeedController::kPercentVbus);
	_armMotor->Set(0);
}

// ==========================================================================

void ArmSub::DisplayPosition() {
	SmartDashboard::PutNumber("Arm Position", _armMotor->GetPosition() - _offset);
}

// ==========================================================================

void ArmSub::Reset() {
	_armMotor->SetPosition(0.0);
}

// ==========================================================================

void ArmSub::SetOffset(double offset) {
	_offset = offset;
}

// ==========================================================================

void ArmSub::Stop() {
	//_armMotor->Set(0);
}

// ==========================================================================

void ArmSub::StowArm() {
	_armMotor->SetControlMode(CANSpeedController::kPosition);
	_armMotor->SetSetpoint(0.0 + _offset);
}

// ==========================================================================
