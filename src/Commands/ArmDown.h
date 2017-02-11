#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ArmDown : public frc::Command {
public:
	ArmDown();

	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

// ==========================================================================
