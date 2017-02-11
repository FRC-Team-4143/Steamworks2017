#ifndef SHOOTER_H
#define SHOOTER_H
#include "Commands/Subsystem.h"
#include "WPILib.h"


class Shooter: public Subsystem {
public:

	CANTalon *shooterLeftFront;
	CANTalon *shooterLeftBack;
	CANTalon *shooterRightFront;
	CANTalon *shooterRightBack;

	Shooter();
	void InitDefaultCommand();
	void shootFront();
	void shootBack();
	void stopFront();
	void stopBack();
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
