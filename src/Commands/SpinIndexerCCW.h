#pragma once

#include "Commands/Command.h"

class SpinIndexerCCW : public frc::Command {
public:
	SpinIndexerCCW();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
