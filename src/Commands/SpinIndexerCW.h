#pragma once

#include "Commands/Command.h"

class SpinIndexerCW : public frc::Command {
public:
	SpinIndexerCW();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
