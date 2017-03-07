#pragma once

#include "Commands/Command.h"

class DefaultIndexer : public frc::Command {
public:
	DefaultIndexer();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};
