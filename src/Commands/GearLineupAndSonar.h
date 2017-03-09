#pragma once

#include <Commands/Command.h>
#include "Modules/VisionSink.h"
#include "Modules/VisionSource.h"

// ==========================================================================

class GearLineupAndSonar : public frc::Command {
public:
	GearLineupAndSonar(double y, double x, double twist, double seconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double _timeoutSeconds;
	int _counter;
	int _waiting;
	int _waitingCounter;
	double _angle;
	double _p;
	double _i;
	double _d;
	double _tol;
	double _center;
	double _x;
	double _y;
	double _twist;
	double _seconds;

	void _Cleanup();
};

// ==========================================================================
