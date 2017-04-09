#include "Commands/GearLineup.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

GearLineup::GearLineup()
: frc::Command("GearLineup"),
	_timeoutSeconds(2.5),
	_counter(0), _waiting(0), _waitingCounter(0),
	_angle(0),
	_tol(0),
	_center(0) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void GearLineup::Initialize() {
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

void GearLineup::Execute() {

	auto pixels = Robot::visionBridge->GetGearPosition() - SmartDashboard::GetNumber("vision center", 0);
	pixels *= 0.1;
	SmartDashboard::PutNumber("GearPixels", pixels);

		if (pixels < -_tol) {
			Robot::driveTrain->Crab(0, 0, 0.2, false);
		} else if (pixels > _tol){
			Robot::driveTrain->Crab(0, 0, -0.2, false);
		} else {
			Robot::driveTrain->Crab(0, 0, 0, false);
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

bool GearLineup::IsFinished() {
	return IsTimedOut() || _counter > 50;
}

// ==========================================================================

void GearLineup::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void GearLineup::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void GearLineup::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
	Robot::driveTrain->disableSpeedControl();
	_counter = 0;
}

// ==========================================================================
