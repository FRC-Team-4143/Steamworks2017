#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ShooterSlower : public frc::Command {
public:
	ShooterSlower();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
