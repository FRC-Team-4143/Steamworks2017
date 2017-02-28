#pragma once

#include "Commands/Command.h"

class ScriptIndex : public frc::Command {
public:
	ScriptIndex(float speed, float timeout);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	float _timeout;
	float _speed;
};
