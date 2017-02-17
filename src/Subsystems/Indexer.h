#ifndef Indexer_H
#define Indexer_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>

class Indexer : public Subsystem {



private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

public:
	Indexer();
	void InitDefaultCommand();
	void SpinCW();
	void SpinCCW();
	void SpinBall();
	void Stop();
	CANTalon *indexMotor;
};

#endif  // Indexer_H
