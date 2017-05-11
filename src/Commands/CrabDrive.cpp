#include "Commands/CrabDrive.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

CrabDrive::CrabDrive()
: frc::Command("CrabDrive") {
	LOG(GetName() + "::ctor");

	Requires(Robot::driveTrain);
}

// ==========================================================================

void CrabDrive::Initialize() {
	Robot::driveTrain->CrabInit();
}

// ==========================================================================

void CrabDrive::Execute() {
	float x = 0;
	float y = 0;
	float z = 0;
	if (!DriverStation::GetInstance().IsAutonomous()) {
		x = Robot::oi->GetJoystickX();
		y = Robot::oi->GetJoystickY();
		z = Robot::oi->GetJoystickZ();
	}

	x *= (x < 0 ? -x: x);
	z *= (z < 0 ? -z: z);
	if (fabs(y) > 0.5)
		z *= 0.75;
	y *= (y < 0 ? -y: y);



	Robot::driveTrain->Crab(z, -y, x, true); //ENABLE THIS
}

// ==========================================================================

bool CrabDrive::IsFinished() {
	return false;
}

// ==========================================================================

void CrabDrive::End() {
}

// ==========================================================================

void CrabDrive::Interrupted() {
	End();
}

// ==========================================================================
