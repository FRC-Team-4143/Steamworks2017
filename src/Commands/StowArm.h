#pragma once

#include <Commands/Command.h>

// ==========================================================================

class StowArm : public frc::Command {
public:
	StowArm();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
