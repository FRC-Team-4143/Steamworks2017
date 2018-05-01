#pragma once

#include "ctre/phoenix.h"
#include <Commands/Subsystem.h>

class Turret : public Subsystem {
private:
	//CANTalon *indexMotor;
	WPI_TalonSRX *turretMotor;

public:
	Turret();
	void InitDefaultCommand();
	void SetSpeed(double speed);
	void Stop();

	float startingPosition;
};
