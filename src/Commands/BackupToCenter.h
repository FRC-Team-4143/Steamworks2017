#pragma once

#include <Commands/Command.h>

// ==========================================================================

class BackupToCenter : public frc::Command {
public:
	BackupToCenter();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
