#pragma once

#include <Commands/Subsystem.h>
#include <ctre/phoenix.h>

class Pickup : public Subsystem {
	WPI_TalonSRX *pickup;

private:

public:
	Pickup();
	void InitDefaultCommand();
	void In();
	void Out();
	void Stop();
};
