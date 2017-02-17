#ifndef SHOOTER_H
#define SHOOTER_H
#include "Commands/Subsystem.h"
#include "WPILib.h"


class Shooter: public Subsystem {
public:
	float targetShooterSpeed;

	CANTalon *shooterMotor1;
	CANTalon *shooterMotor2;

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
    void Feed() {}
    bool IsAtShootingSpeed() { return true;}
    bool SeesBall() { return true;}
    void ShootFront() {}
    void StopFeed() {}
    void ShootBack() {}
    void DeFeed() {}
};

#endif
