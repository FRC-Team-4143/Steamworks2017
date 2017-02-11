#pragma once

#include <Commands/Command.h>

// ==========================================================================

class LowGoalShoot : public frc::Command {
public:
	LowGoalShoot(double timeoutSeconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	int _counter;
	double _timeoutSeconds;
};

// ==========================================================================
