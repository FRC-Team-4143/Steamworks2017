#pragma once

#include "Commands/Subsystem.h"
#include "WPILib.h"

class Shooter: public Subsystem {
public:
	float targetShooterSpeed;

	WPI_TalonSRX *shooterMotor1;
	WPI_TalonSRX *shooterMotor2;

	Shooter();
	void InitDefaultCommand();
	void shootFront();
	void shootBack();
	void stopFront();
	void stopBack();
	void IncreaseTargetRpm();
	void DecreaseTargetRpm();
	void shootDefault(float right, float left);
	double getVelocity();
	void readValues();
	int calculateSpeed();
    void Feed() {}
    bool IsAtShootingSpeed() { return true;}
    bool SeesBall() { return true;}
    void ShootFront() {}
    void StopFeed() {}
    void ShootBack() {}
    void DeFeed() {}
};
