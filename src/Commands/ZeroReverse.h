#pragma once

#include "Commands/Command.h"

class ZeroReverse : public frc::Command {
public:
	ZeroReverse();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
