#ifndef Indexer_H
#define Indexer_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>
#include <WPILib.h>
#include <Timer.h>

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
	bool IsJammed();
	void testJamShooter();
	void readPDP();
	void setSpeed(double speed);

	CANTalon *indexMotor;
	PowerDistributionPanel *pdp;

	Timer *timer;
	bool indexJammed;
	double reverseTime;
	double jamPosition;
	bool loadingOne;

};

#endif  // Indexer_H
