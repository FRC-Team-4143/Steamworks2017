#pragma once

#include "CANTalon.h"
#include <Commands/Subsystem.h>

class Turret : public Subsystem {
private:
	//CANTalon *indexMotor;
	CANTalon *turretMotor;

public:
	Turret();
	void InitDefaultCommand();
	void SetSpeed(double speed);
	void Stop();

	float startingPosition;
};
