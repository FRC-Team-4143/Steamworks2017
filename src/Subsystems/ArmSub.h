#pragma once

#include <Commands/Subsystem.h>
#include <CANTalon.h>

// ==========================================================================

class ArmSub : public frc::Subsystem {
public:
	ArmSub();

	void InitDefaultCommand();

	double GetRawPosition() const;
	void ArmDown();
	void ArmUp();
	void DisablePositionControl();
	void DisplayPosition();
	void Reset();
	void SetOffset(double offset);
	void Stop();
	void StowArm();

private:
	CANTalon* _armMotor;
	double _offset;
};

// ==========================================================================
