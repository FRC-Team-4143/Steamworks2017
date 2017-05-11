#include "BoilerLineup.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

BoilerLineup::BoilerLineup(int offset, int side)
: frc::Command("BoilerLineup"), _offset(offset), _side(side),
	_timeoutSeconds(1),
	_counter(0), _waiting(0), _waitingCounter(0),
	_angle(0),
	_tol(0),
	_center(0) {
	Requires(Robot::turret);
}

// ==========================================================================

void BoilerLineup::Initialize() {
	LOG(GetName() + "::Initialize");

	_counter = 0;
	_waiting = 0;
	_waitingCounter = 0;
	_angle = Robot::gyroSub->PIDGet();
	_angle /= 90.;
	_angle = floor(_angle + .5); // round
	_angle *= 90.;
	_tol = SmartDashboard::GetNumber("vision tol", 5);
	_center = SmartDashboard::GetNumber("vision center", 0.0);
	//Robot::driveTrain->enablePositionControl();
	SetTimeout(_timeoutSeconds);
}

// ==========================================================================

void BoilerLineup::Execute() {
/*
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
*/
	//SmartDashboard::PutBoolean("Waiting", _waiting);
	//SmartDashboard::PutNumber("Waiting counter", _waitingCounter);

	auto pixels = Robot::visionBridge->GetBoilerPosition() - SmartDashboard::GetNumber("vision center", 0) - _offset;
	if (std::abs(pixels) < _tol) {
		_counter++;
	}
	else {
		_counter = 0;
	}
	pixels *= 0.1;
	SmartDashboard::PutNumber("Pixels", pixels);

	//SmartDashboard::PutNumber("Front Left Drive Position", flPos);
	//auto desiredAngle = SmartDashboard::GetNumber("Twist Angle", 0);
	//auto angleError = desiredAngle + pixels;
	//angleError *= (101.3 / 9.9 / 360);

	//if (!_waiting) {
		if (pixels < -_tol) {
			Robot::turret->SetSpeed(0.5);
			_counter = 0;
		} else if (pixels > _tol){
			Robot::turret->SetSpeed(-0.5);
			_counter = 0;
		} else {
			Robot::turret->SetSpeed(0);
			_counter++;
		}
	//}

	/*_waiting++;
	if (_waiting > 20) {
		_waiting = 0;
	}*/

	//SmartDashboard::PutNumber("Angle Error", angleError);
	//Robot::driveTrain->PositionModeTwist(0);
}

// ==========================================================================

bool BoilerLineup::IsFinished() {
	return false; //IsTimedOut() || _counter > 5;
}

// ==========================================================================

void BoilerLineup::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void BoilerLineup::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void BoilerLineup::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
	Robot::driveTrain->disableSpeedControl();
	_counter = 0;
}

// ==========================================================================
