#include "Commands/SetWinchPosition.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

SetWinchPosition::SetWinchPosition(int pos, bool holdPos, double timeoutSeconds)
: _pos(pos), _holdPos(holdPos), _timeoutSeconds(timeoutSeconds) {
	Requires(Robot::winchSub);
}

// ==========================================================================

void SetWinchPosition::Initialize() {
	LOG("SetWinchPosition::Initialize");
	SetTimeout(_timeoutSeconds);
	if (Robot::winchSub->angleSensor) {
		if (_pos == 0) {
			Robot::winchSub->SetPosition(SmartDashboard::GetNumber("Winch 0", 3.2));
		}
		else if (_pos == 1) {
			Robot::winchSub->SetPosition(SmartDashboard::GetNumber("Winch 1", 3.0));
		}
		else if (_pos == 2) {
			Robot::winchSub->SetPosition(SmartDashboard::GetNumber("Winch 2", 3.4));
		}
		else if (_pos == 3) {
			if (Robot::visionBridge->GetDistance() > SmartDashboard::GetNumber("Long Shot Cutoff", 100)) {
				Robot::winchSub->SetPosition(SmartDashboard::GetNumber("Winch 0", 3.2));
			}
			else {
				Robot::winchSub->SetPosition(SmartDashboard::GetNumber("Winch 2", 3.4));
			}
		}
	}
	else {
		if (_pos == 0) {
			Robot::winchSub->SetPosition(0.0);
		}
		else if (_pos == 1) {
			Robot::winchSub->SetPosition(0.1);
		}
		else if (_pos == 2) {
			Robot::winchSub->SetPosition(0.8);
		}
	}
}

// ==========================================================================

void SetWinchPosition::Execute() {
}

// ==========================================================================

bool SetWinchPosition::IsFinished() {
	return IsTimedOut() || Robot::winchSub->OnTarget();
}

// ==========================================================================

void SetWinchPosition::End() {
	LOG("SetWinchPosition::End");
	if (!_holdPos) {
		Robot::winchSub->DisablePositionControl();
	}
}

// ==========================================================================

void SetWinchPosition::Interrupted() {
	End();
}

// ==========================================================================
