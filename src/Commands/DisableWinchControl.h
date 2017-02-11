#pragma once

#include <Commands/Command.h>

// ==========================================================================

class DisableWinchControl : public frc::Command {
public:
	DisableWinchControl();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
