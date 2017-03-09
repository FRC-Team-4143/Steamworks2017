#include "Commands/GearLineupAndSonar.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

GearLineupAndSonar::GearLineupAndSonar(double y, double x, double twist, double seconds)
: frc::Command("GearLineupAndSonar"),
	_timeoutSeconds(5),
	_counter(0), _waiting(0), _waitingCounter(0),
	_angle(0),
	_p(0), _i(0), _d(0), _tol(0),
	_center(0),
	_x(x),
	_y(y),
	_twist(twist),
	_seconds(seconds) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void GearLineupAndSonar::Initialize() {
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
	_tol = SmartDashboard::GetNumber("vision tol", 5);
	_center = SmartDashboard::GetNumber("vision center", 0.0);
	//Robot::driveTrain->enablePositionControl();
	SetTimeout(_timeoutSeconds);
}

// ==========================================================================

void GearLineupAndSonar::Execute() {
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

	auto pixels = Robot::visionBridge->GetGearPosition() - SmartDashboard::GetNumber("vision center", 0);
	pixels *= 0.1;
	SmartDashboard::PutNumber("GearPixels", pixels);

	//SmartDashboard::PutNumber("Front Left Drive Position", flPos);
	//auto desiredAngle = SmartDashboard::GetNumber("Twist Angle", 0);
	//auto angleError = desiredAngle + pixels;
	//angleError *= (101.3 / 9.9 / 360);
	/*if (!_waiting) {
		RobotMap::driveTrainFrontLeftDrive->SetSetpoint(flPos - angleError);
		RobotMap::driveTrainFrontRightDrive->SetSetpoint(frPos - angleError);
		RobotMap::driveTrainRearLeftDrive->SetSetpoint(rlPos - angleError);
		RobotMap::driveTrainRearRightDrive->SetSetpoint(rrPos - angleError);
	}*/
	//if (!_waiting) {
		if (pixels < -_tol) {
			Robot::driveTrain->GyroCrab(_twist, _y, 0.2, false);
		} else if (pixels > _tol){
			Robot::driveTrain->GyroCrab(_twist, _y, -0.2, false);
		} else {
			Robot::driveTrain->GyroCrab(_twist, _y, 0, false);
		}
		if (fabs(pixels) < _tol)
			_counter++;
		SmartDashboard::PutNumber("Autonomous Counter", _counter);

	//}
	_waiting++;
	if (_waiting > 20) {
		_waiting = 0;
	}

	//SmartDashboard::PutNumber("Angle Error", angleError);
	//Robot::driveTrain->PositionModeTwist(0);
}

// ==========================================================================

bool GearLineupAndSonar::IsFinished() {
	return IsTimedOut() || RobotMap::sonar->GetAverageVoltage() < 0.3;
}

// ==========================================================================

void GearLineupAndSonar::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void GearLineupAndSonar::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void GearLineupAndSonar::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
	Robot::driveTrain->disableSpeedControl();
	_counter = 0;
}

// ==========================================================================
