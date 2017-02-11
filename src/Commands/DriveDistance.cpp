#include "Commands/DriveDistance.h"
#include "Robot.h"

// ==========================================================================

DriveDistance::DriveDistance(float driveAngle, float speed, float twistAngle, float distance)
: _driveAngle(driveAngle), _speed(speed), _twistAngle(twistAngle), _distance(distance) {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void DriveDistance::Initialize() {
	Robot::driveTrain->zeroDistanceEncoders();
}

// ==========================================================================

void DriveDistance::Execute() {
	Robot::driveTrain->Crab(_twistAngle, _driveAngle, _speed, false);
}

// ==========================================================================

bool DriveDistance::IsFinished() {
	return Robot::driveTrain->getDistanceEncodersValues() >= _distance;
}

// ==========================================================================

void DriveDistance::End() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================

void DriveDistance::Interrupted() {
	End();
}

// ==========================================================================
