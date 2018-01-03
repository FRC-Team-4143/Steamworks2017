#pragma once

#include <Commands/Subsystem.h>
#include <ctrlib/CANTalon.h>

class Climber : public Subsystem {
	CANTalon *climbingMotor;
	CANTalon *climbingMotor2;


private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Climber();
	void InitDefaultCommand();
	void climb(float speed);
	void reverseClimb(float speed);
	void stopClimb();
};
