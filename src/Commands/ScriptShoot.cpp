#include "ScriptShoot.h"
#include "ScriptStartShooter.h"
#include "ScriptStopShooter.h"
#include "ScriptIndex.h"

ScriptShoot::ScriptShoot(int speed, float timeout) {
	AddSequential(new ScriptStartShooter(speed), 1);
	AddSequential(new ScriptIndex(0.65, timeout));
	AddSequential(new ScriptStopShooter());
}
