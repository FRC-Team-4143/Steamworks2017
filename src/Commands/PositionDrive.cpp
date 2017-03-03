#include "Commands/PositionDrive.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

PositionDrive::PositionDrive(int offset, int side)
: frc::Command("PositionDrive"), _offset(offset), _side(side),
	_timeoutSeconds(10),
	_counter(0), _waiting(0), _waitingCounter(0),
	_angle(0),
	_p(0), _i(0), _d(0), _tol(0),
	_center(0) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void PositionDrive::Initialize() {
	LOG(GetName() + "::Initialize");

	_counter = 0;
	_waiting = 0;
	_waitingCounter = 0;
	_angle = Robot::gyroSub->PIDGet();
	_angle /= 90.;
	_angle = floor(_angle + .5); // round
	_angle *= 90.;
	_p = SmartDashboard::GetNumber("vision P", .3);
	_i = SmartDashboard::GetNumber("vision I", .3);
	_d = SmartDashboard::GetNumber("vision D", .3);
	_tol = SmartDashboard::GetNumber("vision tol", 15);
	_center = SmartDashboard::GetNumber("vision center", 25.0);
	Robot::driveTrain->enablePositionControl();
	SetTimeout(_timeoutSeconds);
}

// ==========================================================================

void PositionDrive::Execute() {
	auto flPos = RobotMap::driveTrainFrontLeftDrive->GetPosition();
	auto frPos = RobotMap::driveTrainFrontRightDrive->GetPosition();
	auto rlPos = RobotMap::driveTrainRearLeftDrive->GetPosition();
	auto rrPos = RobotMap::driveTrainRearRightDrive->GetPosition();

	if (std::fabs(flPos - RobotMap::driveTrainFrontLeftDrive->GetSetpoint()) < 0.002 ||
			std::fabs(frPos - RobotMap::driveTrainFrontRightDrive->GetSetpoint()) < 0.002 ||
			std::fabs(rlPos - RobotMap::driveTrainRearLeftDrive->GetSetpoint()) < 0.002 ||
			std::fabs(rrPos - RobotMap::driveTrainRearRightDrive->GetSetpoint()) < 0.002) {
		_waitingCounter++;
	}
	else {
		_waitingCounter = 0;
	}

	if (_waitingCounter > 5) {
		_waiting = 0;
		_waitingCounter = 0;
	}

	SmartDashboard::PutBoolean("Waiting", _waiting);
	SmartDashboard::PutNumber("Waiting counter", _waitingCounter);

	auto pixels = Robot::visionBridge->GetBoilerPosition() - SmartDashboard::GetNumber("vision center", 40) - _offset;
	if (std::abs(pixels) < _tol) {
		_counter++;
	}
	else {
		_counter = 0;
	}
	pixels *= 0.1;
	SmartDashboard::PutNumber("Pixels", pixels);

	SmartDashboard::PutNumber("Front Left Drive Position", flPos);
	auto desiredAngle = SmartDashboard::GetNumber("Twist Angle", 0);
	auto angleError = desiredAngle + pixels;
	angleError *= (101.3 / 9.9 / 360);
	if (!_waiting) {
		RobotMap::driveTrainFrontLeftDrive->SetSetpoint(flPos - angleError);
		RobotMap::driveTrainFrontRightDrive->SetSetpoint(frPos - angleError);
		RobotMap::driveTrainRearLeftDrive->SetSetpoint(rlPos - angleError);
		RobotMap::driveTrainRearRightDrive->SetSetpoint(rrPos - angleError);
	}
	_waiting++;
	if (_waiting > 20) {
		_waiting = 0;
	}

	SmartDashboard::PutNumber("Angle Error", angleError);
	Robot::driveTrain->PositionModeTwist(0);
}

// ==========================================================================

bool PositionDrive::IsFinished() {
	return IsTimedOut() || _counter > 5;
}

// ==========================================================================

void PositionDrive::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void PositionDrive::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void PositionDrive::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
	Robot::driveTrain->disableSpeedControl();
	_counter = 0;
}

// ==========================================================================
