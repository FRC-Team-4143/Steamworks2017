#pragma once

#include "Commands/Command.h"

class PickupIn : public frc::Command {
public:
	PickupIn();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
