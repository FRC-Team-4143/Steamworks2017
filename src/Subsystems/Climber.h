#pragma once

#include <Commands/Subsystem.h>
#include <CANTalon.h>

class Climber : public Subsystem {
	CANTalon *climbingMotor;

private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Climber();
	void InitDefaultCommand();
	void climb();
	void reverseClimb();
	void stopClimb();
};
