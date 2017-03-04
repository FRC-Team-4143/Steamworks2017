#pragma once

#include <Commands/Command.h>
#include "Modules/VisionSink.h"
#include "Modules/VisionSource.h"

// ==========================================================================

class GearLineup : public frc::Command {
public:
	GearLineup();

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

	void _Cleanup();
};

// ==========================================================================
