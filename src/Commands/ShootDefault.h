#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ShootDefault : public frc::Command {
public:
	ShootDefault();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
