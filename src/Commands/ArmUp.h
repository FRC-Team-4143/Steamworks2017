#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ArmUp : public frc::Command {
public:
	ArmUp();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

// ==========================================================================
