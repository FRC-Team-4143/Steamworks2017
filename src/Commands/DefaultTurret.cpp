#include "DefaultTurret.h"
#include "Robot.h"

DefaultTurret::DefaultTurret() {
	Requires(Robot::turret);
	Requires(Robot::indexer);
}

void DefaultTurret::Initialize() {

}
void DefaultTurret::Execute() {
	Robot::turret->SetSpeed(Robot::oi->GetJoystickX2()*50);
}

bool DefaultTurret::IsFinished() {
	return false;
}

void DefaultTurret::End() {
	Robot::turret->Stop();
}

void DefaultTurret::Interrupted() {
	End();
}
