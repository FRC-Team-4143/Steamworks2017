#ifndef DefaultTurret_H
#define DefaultTurret_H

#include "Commands/Command.h"

class DefaultTurret : public frc::Command {
public:
	DefaultTurret();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DefaultTurret_H
