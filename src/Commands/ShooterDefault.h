#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ShooterDefault : public frc::Command {
public:
	ShooterDefault();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
