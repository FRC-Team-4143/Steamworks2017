#pragma once

#include <Commands/Command.h>
#include <CANTalon.h>

// ==========================================================================

class RunMotor : public frc::Command {
public:
	RunMotor(CANTalon* motor);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	CANTalon* _motor;
};

// ==========================================================================
