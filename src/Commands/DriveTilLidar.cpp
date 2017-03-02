#include "Commands/DriveTilLidar.h"
#include "Robot.h"

// ==========================================================================

DriveTilLidar::DriveTilLidar(double target, double y, double x, double twist, double seconds)
: _target(target), _x(x), _y(y), _twist(twist), _seconds(seconds) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void DriveTilLidar::Initialize() {
	SetTimeout(_seconds);
}

// ==========================================================================

void DriveTilLidar::Execute() {
	Robot::driveTrain->GyroCrab(_twist, _y, _x, false);
}

// ==========================================================================

bool DriveTilLidar::IsFinished() {
	return (fabs(Robot::driveTrain->lidarDistance - _target) < 5) ||
		IsTimedOut();
}

// ==========================================================================

void DriveTilLidar::End() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================

void DriveTilLidar::Interrupted() {
	End();
}

// ==========================================================================
