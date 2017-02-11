#include "Subsystems/Shooter.h"
#include <CANTalon.h>
#include "Commands/ShootDefault.h"
#include "RobotMap.h"

const int SHOOTING_SPEED_RPM = 5000;

// ==========================================================================

Shooter::Shooter()
: frc::Subsystem("Shooter"),
	_leftFront(RobotMap::leftFront),
	_leftRear(RobotMap::leftRear),
	_rightFront(RobotMap::rightFront),
	_rightRear(RobotMap::rightRear),
	_feeder(RobotMap::feeder),
	_feederSensor(RobotMap::feederSensor) {

	_leftFront->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	_rightFront->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	/*
	leftFront->SetControlMode(CANSpeedController::kSpeed);
	leftFront->ConfigNominalOutputVoltage(0.0, 0.0);
	leftFront->ConfigPeakOutputVoltage(12.0, -12.0);
	leftFront->SetSensorDirection(true);
	leftFront->SetP(0.05);
	leftFront->SetI(0.0);
	leftFront->SetD(0.0);
	leftFront->SetF(0.03);

	rightFront->SetControlMode(CANSpeedController::kSpeed);
	rightFront->ConfigNominalOutputVoltage(0.0, 0.0);
	rightFront->ConfigPeakOutputVoltage(12.0, -12.0);
	rightFront->SetSensorDirection(true);
	rightFront->SetP(0.05);
	rightFront->SetI(0.0);
	rightFront->SetD(0.0);
	rightFront->SetF(0.03);
	*/
}

// ==========================================================================

double Shooter::GetVelocity() const {
	return _leftFront->GetSpeed();
}

// ==========================================================================

void Shooter::InitDefaultCommand() {
	SetDefaultCommand(new ShootDefault());
}

// ==========================================================================

bool Shooter::IsAtShootingSpeed() {
	return std::abs(_leftFront->GetSpeed()) > SHOOTING_SPEED_RPM;
}

// ==========================================================================

bool Shooter::SeesBall() {
	return _feederSensor->GetAverageVoltage() > 4.0;
}

// ==========================================================================

void Shooter::Shoot(float right, float left) {
	if (right > 0.1) {
		ShootFront();
		ShootBack();
	}
	else {
		StopFront();
		StopBack();
	}

	if ((left > 0.1 && SeesBall()) || (left > 0.1 && right > 0.1)) {
		Feed();
	}
	else {
		StopFeed();
	}
}

// ==========================================================================

void Shooter::ShootFront() {
	_leftFront->Set(1);
	_rightFront->Set(-1);
	/*
	leftFront->SetControlMode(CANSpeedController::kSpeed);
	leftFront->Set(SHOOTING_SPEED_RPM);
	_rightFront->SetControlMode(CANSpeedController::kSpeed);
	_rightFront->Set(-SHOOTING_SPEED_RPM);
	*/
}

// ==========================================================================

void Shooter::ShootBack() {
	_leftRear->Set(-1);
	_rightRear->Set(1);
}

// ==========================================================================

void Shooter::StopFront() {
	_leftFront->SetControlMode(CANSpeedController::kPercentVbus);
	_leftFront->Set(0);
	_rightFront->SetControlMode(CANSpeedController::kPercentVbus);
	_rightFront->Set(0);
}

// ==========================================================================

void Shooter::StopBack() {
	_leftRear->Set(0);
	_rightRear->Set(0);
}

// ==========================================================================

void Shooter::Feed() {
	_feeder->Set(-0.7);
}

// ==========================================================================

void Shooter::DeFeed() {
	_feeder->Set(0.5);
}

// ==========================================================================

void Shooter::StopFeed() {
	_feeder->Set(0);
}

// ==========================================================================
