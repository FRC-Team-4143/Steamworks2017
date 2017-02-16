#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ShooterFaster : public frc::Command {
public:
	ShooterFaster();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
