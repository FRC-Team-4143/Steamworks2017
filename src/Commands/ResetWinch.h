#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ResetWinch : public frc::Command {
public:
	ResetWinch();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
