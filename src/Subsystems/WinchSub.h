#pragma once

#include <Commands/Subsystem.h>
#include <WPILib.h>
#include <CANTalon.h>

// ==========================================================================

class WinchSub : public frc::Subsystem {
public:
	WinchSub();

	void InitDefaultCommand();

	void DisablePositionControl();
	bool OnTarget() const;
	void ReportPosition();
	void SetPosition(double pos);
	bool ShooterIsRaised() const;

	void Climb();
	void ClimbReverse();
	void Reset();
	void StopClimb();

	const static bool angleSensor; // TODO - Make this private

private:
	CANTalon* _motor;
	AnalogInput* _shooterSensor;
	PIDController* _shooter;
	bool _shooterRaised;
};

// ==========================================================================
