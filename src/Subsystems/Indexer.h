#ifndef Indexer_H
#define Indexer_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>

class Indexer : public Subsystem {

	CANTalon *indexMotor;

private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Indexer();
	void InitDefaultCommand();
	void SpinCW();
	void SpinCCW();
	void Stop();
};

#endif  // Indexer_H
