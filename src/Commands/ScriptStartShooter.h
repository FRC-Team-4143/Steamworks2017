#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ScriptStartShooter : public frc::Command {
public:
	ScriptStartShooter(int speed);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	int _speed;
};

// ==========================================================================
