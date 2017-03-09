#include "Commands/GyroCrab.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

GyroCrab::GyroCrab()
: frc::Command("Crab Drive"), _angle(0) {
	LOG(GetName() + "::ctor");

	Requires(Robot::driveTrain);
	flag = false;
}

// ==========================================================================

void GyroCrab::Initialize() {
	_angle = Robot::gyroSub->PIDGet();
	_angle /= 90;
	_angle = floor(_angle + .5); // round
	_angle *= 90;

}

// ==========================================================================

void GyroCrab::Execute() {
	auto x = Robot::oi->GetJoystickX();
	auto y = Robot::oi->GetJoystickY();
	if (Robot::oi->GetRightTrigger() > 0.5 && !flag){
		flag = true;
		_angle += 30;
	} else if (Robot::oi->GetLeftTrigger() > 0.5 && !flag){
		flag = true;
		_angle -= 30;
	} else if (!(Robot::oi->GetRightTrigger() > 0.5) && !(Robot::oi->GetLeftTrigger() > 0.5)){
		flag = false;
	}
	Robot::driveTrain->GyroCrab(_angle, -y, x, true);
}

// ==========================================================================

bool GyroCrab::IsFinished() {
	return false;
}

// ==========================================================================

void GyroCrab::End() {
}

// ==========================================================================

void GyroCrab::Interrupted() {
	End();
}

// ==========================================================================
