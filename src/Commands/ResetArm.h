#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ResetArm : public frc::Command {
public:
	ResetArm();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
