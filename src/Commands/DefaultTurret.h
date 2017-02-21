#pragma once

#include "Commands/Command.h"

class DefaultTurret : public frc::Command {
public:
	DefaultTurret();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
