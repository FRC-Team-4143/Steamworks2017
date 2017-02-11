#pragma once

#include <Commands/Command.h>

// ==========================================================================

class Shoot : public frc::Command {
public:
	Shoot(double timeoutSeconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

public:
	double _timeoutSeconds;
};

// ==========================================================================
