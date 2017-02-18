#pragma once

#include <Commands/Command.h>

// ==========================================================================

class TestJamShooter : public frc::Command {
public:
	TestJamShooter();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

// ==========================================================================
