#include "ScriptShoot.h"
#include "ScriptStartShooter.h"
#include "ScriptStopShooter.h"
#include "ScriptIndex.h"
#include "PositionDrive.h"

ScriptShoot::ScriptShoot(int speed, float timeout) {
	//AddSequential(new PositionDrive(0, 0));
	AddSequential(new ScriptStartShooter(speed), 0.5);
	AddSequential(new ScriptIndex(0.65, timeout));
	AddSequential(new ScriptStopShooter());
}
