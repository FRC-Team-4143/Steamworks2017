#include "Subsystems/WinchSub.h"
#include <SmartDashboard/SmartDashboard.h>
#include "RobotMap.h"

const bool WinchSub::angleSensor = true;

// ==========================================================================

WinchSub::WinchSub()
:	frc::Subsystem("ExampleSubsystem"),
	_motor(nullptr), _shooterSensor(nullptr), _shooter(nullptr), _shooterRaised(false) {
	_motor = RobotMap::winchMotor;
	_shooterSensor = RobotMap::winchPot;
	if (angleSensor) {
		_shooter = RobotMap::shooterWinch;
	}
	else {
		_motor->SetControlMode(CANSpeedController::kPosition);
		_motor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
		_motor->SetP(0.8);
		_motor->SetI(0.0);
		_motor->SetD(0.05);
		_motor->SetF(0.0);
		_motor->Enable();
		_motor->ConfigPeakOutputVoltage(6.0, -6.0);
	}
}

// ==========================================================================

void WinchSub::InitDefaultCommand() {
	// SetDefaultCommand(new MySpecialCommand());
}

// ==========================================================================

void WinchSub::DisablePositionControl() {
	if (angleSensor) {
		_shooter->Disable();
	}
	else {
		_motor->SetControlMode(CANSpeedController::kPercentVbus);
	}
}

// ==========================================================================

bool WinchSub::OnTarget() const {
	if (angleSensor) {
		return _shooter->OnTarget();
	}
	return false;
}

// ==========================================================================

void WinchSub::ReportPosition() {
	SmartDashboard::PutNumber("Winch Position", _motor->GetPosition());
	SmartDashboard::PutNumber("Winch Angle", _shooterSensor->GetAverageVoltage());
}

// ==========================================================================

void WinchSub::SetPosition(double pos) {
	if (angleSensor) {
		_shooter->Enable();
		_shooter->SetSetpoint(pos);
	}
	else {
		_motor->SetControlMode(CANSpeedController::kPosition);
		_motor->SetSetpoint(pos);
	}

	if (pos < 0.2) {
		_shooterRaised = true;
	}
	else {
		_shooterRaised = false;
	}
}

// ==========================================================================

bool WinchSub::ShooterIsRaised() const {
	return _shooterRaised;
}

// ==========================================================================

void WinchSub::Climb() {
	_motor->SetControlMode(CANSpeedController::kPercentVbus);
	_motor->Set(1);
}

// ==========================================================================

void WinchSub::ClimbReverse() {
	_motor->SetControlMode(CANSpeedController::kPercentVbus);
	_motor->Set(-1);
}

// ==========================================================================

void WinchSub::Reset() {
	_motor->SetPosition(0.0);
}

// ==========================================================================

void WinchSub::StopClimb() {
	_motor->Set(0);
}

// ==========================================================================
