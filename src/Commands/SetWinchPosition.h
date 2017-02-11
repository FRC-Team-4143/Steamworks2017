#pragma once

#include <Commands/Command.h>

// ==========================================================================

class SetWinchPosition : public frc::Command {
public:
	SetWinchPosition(int pos, bool holdPos, double timeoutSeconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	int _pos;
	bool _holdPos;
	double _timeoutSeconds;
};

// ==========================================================================
