#pragma once

#include <Commands/Subsystem.h>
#include <CANTalon.h>
#include <WPILib.h>
#include <Timer.h>

class Indexer : public Subsystem {
private:
	CANTalon *indexMotor;
	PowerDistributionPanel *pdp;
	Timer *timer;
	bool indexJammed;
	double reverseTime;
	double jamPosition;
	bool _loadingOne;

public:
	Indexer();
	void InitDefaultCommand();
	void SpinCW();
	void SpinCW(float speed);
	void SpinCCW();
	void SpinBall();
	void SetLoadingOne(bool loadingOne);
	void Stop();
	bool IsJammed();
	void TestJamShooter();
	void ReadPDP();
	void SetSpeed(double speed);
	void ZeroTimer();
	double GetPosition() const;
	double GetReverseTime() const;
	double GetTimer() const;
	bool IsIndexJammed() const;
};
