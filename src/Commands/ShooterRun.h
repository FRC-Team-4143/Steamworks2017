#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ShooterRun : public frc::Command {
public:
	ShooterRun();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
