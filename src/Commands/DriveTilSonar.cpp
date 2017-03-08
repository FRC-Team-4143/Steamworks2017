#include "Commands/DriveTilSonar.h"
#include "Robot.h"

// ==========================================================================

DriveTilSonar::DriveTilSonar(double y, double x, double twist, double seconds)
: _x(x), _y(y), _twist(twist), _seconds(seconds) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void DriveTilSonar::Initialize() {
	SetTimeout(_seconds);
}

// ==========================================================================

void DriveTilSonar::Execute() {
	Robot::driveTrain->GyroCrab(_twist, _y, _x, false);
}

// ==========================================================================

bool DriveTilSonar::IsFinished() {
	return (RobotMap::sonar->GetAverageVoltage() < 0.3) || //The 5 will definitely be different
		IsTimedOut();
}

// ==========================================================================

void DriveTilSonar::End() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================

void DriveTilSonar::Interrupted() {
	End();
}

// ==========================================================================
