#pragma once

#include <Commands/Command.h>

// ==========================================================================

class DriveTilLidar : public frc::Command {
public:
	DriveTilLidar(double target, double y, double x, double twist, double seconds);

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double _target;
	double _x;
	double _y;
	double _twist;
	double _seconds;
};

// ==========================================================================
