#ifndef Pickup_H
#define Pickup_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>

class Pickup : public Subsystem {

	CANTalon *pickup;

private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Pickup();
	void InitDefaultCommand();
	void In();
	void Out();
	void Stop();
};

#endif  // Pickup_H
