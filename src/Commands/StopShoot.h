#pragma once

#include <Commands/Command.h>

// ==========================================================================

class StopShoot : public frc::Command {
public:
	StopShoot();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
