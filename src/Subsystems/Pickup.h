#pragma once

#include <Commands/Subsystem.h>
#include <ctrlib/CANTalon.h>

class Pickup : public Subsystem {
	CANTalon *pickup;

private:

public:
	Pickup();
	void InitDefaultCommand();
	void In();
	void Out();
	void Stop();
};
