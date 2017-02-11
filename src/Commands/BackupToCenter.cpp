#include "Commands/BackupToCenter.h"
#include "Robot.h"

// ==========================================================================

BackupToCenter::BackupToCenter() {
	Requires(Robot::driveTrain);
}

// ==========================================================================

void BackupToCenter::Initialize() {
}

// ==========================================================================

void BackupToCenter::Execute() {
	if (Robot::visionBridge->GetDistance() != 0) {
		auto speed = (Robot::visionBridge->GetDistance() - 50) * 0.01;
		speed = std::min(std::max(speed, -0.5), 0.5);
		Robot::driveTrain->FieldCentricCrab(0, speed, 0, false);
	}
}

// ==========================================================================

bool BackupToCenter::IsFinished() {
	auto distance = Robot::visionBridge->GetDistance();
	return (distance > 40 && distance < 60);
}

// ==========================================================================

void BackupToCenter::End() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================

void BackupToCenter::Interrupted() {
	End();
}

// ==========================================================================
