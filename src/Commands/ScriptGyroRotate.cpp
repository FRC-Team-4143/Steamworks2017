#include "Commands/ScriptGyroRotate.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptGyroRotate::ScriptGyroRotate(std::string name, float desiredangle, double power, float seconds)
: frc::Command(name), _desiredangle(desiredangle), _power(power), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f, %f, %f)", desiredangle, power, seconds);
	LOG(GetName() + "::ctor" + szParams);

	Requires(Robot::driveTrain);
}

// ==========================================================================

void ScriptGyroRotate::Initialize() {
	LOG(GetName() + "::Initialize");
	SetTimeout(_seconds);
	//Robot::driveTrain->enableSpeedControl();
}

// ==========================================================================

void ScriptGyroRotate::Execute() {
	Robot::driveTrain->GyroRotate(_desiredangle, _power);
}

// ==========================================================================

bool ScriptGyroRotate::IsFinished() { return IsTimedOut(); }

// ==========================================================================

void ScriptGyroRotate::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void ScriptGyroRotate::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void ScriptGyroRotate::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
	//Robot::driveTrain->disableSpeedControl();
}

// ==========================================================================
