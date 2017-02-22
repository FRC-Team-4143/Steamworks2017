#pragma once

#include "Commands/Command.h"

class FuelLight : public frc::Command {
public:
	FuelLight();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
