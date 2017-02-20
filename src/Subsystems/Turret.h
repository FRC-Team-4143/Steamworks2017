#ifndef Turret_H
#define Turret_H

#include "CANTalon.h"

#include <Commands/Subsystem.h>

class Turret : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Turret();
	void InitDefaultCommand();
	void setSpeed(double speed);
	void Stop();

	CANTalon *indexMotor;
	CANTalon *turretMotor;
};


#endif  // Turret_H
