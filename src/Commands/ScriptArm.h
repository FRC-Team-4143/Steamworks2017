#pragma once

#include <Commands/Command.h>

// ==========================================================================

class ScriptArm : public frc::Command {
public:
	ScriptArm(int pos);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	int _pos;
};

// ==========================================================================
