#include "DefaultTurret.h"
#include "Robot.h"

DefaultTurret::DefaultTurret() {
	Requires(Robot::turret);

}

void DefaultTurret::Initialize() {

}
void DefaultTurret::Execute() {
	float speed = Robot::oi->GetLeftTrigger2() - Robot::oi->GetRightTrigger2();
	if (fabs(speed) < 0.5)
		speed = 0;
	Robot::turret->SetSpeed(speed/2);//-Robot::oi->GetJoystickX2()*SmartDashboard::GetNumber("Turret Max Speed",0));//*50);
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
