#include "DefaultIndexer.h"
#include "Robot.h"

DefaultIndexer::DefaultIndexer() {
	Requires(Robot::indexer);
}

// Called just before this Command runs the first time
void DefaultIndexer::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DefaultIndexer::Execute() {
	Robot::indexer->SetSpeed(-Robot::oi->GetJoystickX2()*SmartDashboard::GetNumber("Turret Max Speed",0));//*50);
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultIndexer::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DefaultIndexer::End() {
	Robot::indexer->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultIndexer::Interrupted() {
	End();
}
