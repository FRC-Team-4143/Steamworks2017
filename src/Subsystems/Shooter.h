#pragma once

#include <Commands/Subsystem.h>
#include <WPILib.h>
#include <CANTalon.h>

// ==========================================================================

class Shooter : public frc::Subsystem {
public:
	Shooter();

	void InitDefaultCommand();

	double GetVelocity() const;
	bool IsAtShootingSpeed();
	bool SeesBall();

	void Shoot(float right, float left);
	void ShootFront();
	void ShootBack();
	void StopFront();
	void StopBack();

	void Feed();
	void DeFeed();
	void StopFeed();

private:
	CANTalon* _leftFront;
	CANTalon* _leftRear;
	CANTalon* _rightFront;
	CANTalon* _rightRear;
	frc::SpeedController* _feeder;
	frc::AnalogInput* _feederSensor;
};

// ==========================================================================
