#include "OneBall.h"
#include "Robot.h"

OneBall::OneBall() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::indexer);

}

void OneBall::Initialize() {
	SetTimeout(1);
	Robot::indexer->SpinBall();
	Robot::indexer->SetLoadingOne(true);
}

void OneBall::Execute() {
}

bool OneBall::IsFinished() {
	return IsTimedOut();
}

void OneBall::End() {
	Robot::indexer->Stop();
	Robot::indexer->SetLoadingOne(false);
}

void OneBall::Interrupted() {
	End();
}
