#pragma once

#include <Commands/Command.h>

// ==========================================================================

class Feed : public frc::Command {
public:
	Feed(double seconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double _seconds;
	bool _seenBall;
};

// ==========================================================================
