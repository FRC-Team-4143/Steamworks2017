#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ClimbReverse : public frc::Command {
public:
	ClimbReverse();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
