#include "DefaultTurret.h"
#include "Robot.h"

DefaultTurret::DefaultTurret() {
	Requires(Robot::turret);
	//Requires(Robot::indexer);
}

// Called just before this Command runs the first time
void DefaultTurret::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DefaultTurret::Execute() {

	Robot::turret->setSpeed(Robot::oi->GetJoystickX2());
	//SmartDashboard::PutNumber("JoystickX", Robot::oi->GetJoystickX2());
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultTurret::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DefaultTurret::End() {

	Robot::turret->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultTurret::Interrupted() {

}
