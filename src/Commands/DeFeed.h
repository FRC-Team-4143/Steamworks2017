#pragma once

#include <Commands/Command.h>

// ==========================================================================

class DeFeed : public frc::Command {
public:
	DeFeed(double timeoutSeconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double _timeoutSeconds;
};

// ==========================================================================
