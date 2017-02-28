#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ScriptStopShooter : public frc::Command {
public:
	ScriptStopShooter();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
