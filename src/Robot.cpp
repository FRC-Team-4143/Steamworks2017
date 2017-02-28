#include "Robot.h"
#include "Commands/BackupToCenter.h"
#include "Commands/DriveDistance.h"
#include "Commands/DriveTilVision.h"
#include "Commands/ScriptCamDrive.h"
#include "Commands/ScriptCommand.h"
#include "Commands/ScriptDrive.h"
#include "Commands/ScriptFieldCentricCrab.h"
#include "Commands/ScriptGyroDrive.h"
#include "Commands/ScriptSleep.h"
#include "Commands/WaitForVision.h"
#include "Commands/ZeroYaw.h"
#include "Commands/ScriptShoot.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"
#include "Modules/ScriptCommandFactory.h"

OI* Robot::oi;
Pickup* Robot::pickup = nullptr;
Indexer* Robot::indexer = nullptr;
Turret* Robot::turret = nullptr;
GyroSub* Robot::gyroSub = nullptr;
DriveTrain* Robot::driveTrain = nullptr;
std::shared_ptr<BasicCameraSub> Robot::basicCameraSub;
VisionBridgeSub* Robot::visionBridge = nullptr;
const bool SHOOTER_AS_MASTER_SLAVE = true;
Shooter* Robot::shooter = nullptr;
Climber* Robot::climber = nullptr;

void Robot::RobotInit() {
	Preferences::GetInstance();
	RobotMap::Initialize();

	ScriptInit();
	SmartDashboard::PutString("ScriptCommand", "S(0.5)");
	SmartDashboard::PutString("ScriptValid", "");
	SmartDashboard::PutNumber("Twist Angle", 0);
/*
	SmartDashboard::PutNumber("vision center", 40.0);
	SmartDashboard::PutNumber("vision P", 0.11); // 0.2
	SmartDashboard::PutNumber("vision I", 0.01); // 0.005 Worked without speed control
	SmartDashboard::PutNumber("vision D", .022); // 0.05
	SmartDashboard::PutNumber("vision tol", 5);
	SmartDashboard::PutNumber("long shot cutoff", 100);
*/
	gyroSub = new GyroSub();
	driveTrain = new DriveTrain();
	pickup = new Pickup();
	indexer = new Indexer();
	turret = new Turret();
	basicCameraSub.reset(new BasicCameraSub("cam0"));
	visionBridge = new VisionBridgeSub();
	climber = new Climber();
	driveTrain->SetWheelbase(26.8, 26.8, 22.8);
	driveTrain->loadWheelOffsets();
	shooter = new Shooter();
	oi = new OI();
}

void Robot::RobotPeriodic() {
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
	driveTrain->Dashboard();
	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetAngle());
	SmartDashboard::PutNumber("Gyro Angle Adjustment", RobotMap::imu->GetAngleAdjustment());
	/*
	SmartDashboard::PutNumber("Vision Position Left", Robot::visionBridge->GetPosition(0));
	SmartDashboard::PutNumber("Vision Position Right", Robot::visionBridge->GetPosition(1));
	SmartDashboard::PutNumber("Vision Distance", Robot::visionBridge->GetDistance());
	*/
	//SmartDashboard::PutNumber("Bottom Velocity", shooter->shooterMotor1->GetSpeed());
	//SmartDashboard::PutNumber("Top Velocity", shooter->shooterMotor2->GetSpeed());
	SmartDashboard::PutNumber("Target Shooter Speed", shooter->targetShooterSpeed);
	//SmartDashboard::PutNumber("Indexer Position", indexer->GetPosition());
}

void Robot::AutonomousInit() {
	printf("Match time start: %f\r\n", DriverStation::GetInstance().GetMatchTime());
	driveTrain->enableSteeringPID();
	RobotMap::imu->ZeroYaw();

	printf("Before new ScriptCommand: %f\r\n", DriverStation::GetInstance().GetMatchTime());
	autonomousCommand = ScriptCommandFactory::GetInstance().GetCommand().release();
	printf("After new ScriptCommand: %f\r\n", DriverStation::GetInstance().GetMatchTime());

	if (autonomousCommand != nullptr) {
		autonomousCommand->Start();
	}

	printf("Match time end of init: %f\r\n", DriverStation::GetInstance().GetMatchTime());
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();

	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetAngle());
	/*
	SmartDashboard::PutNumber("Vision Position Left", Robot::visionBridge->GetPosition(0));
	SmartDashboard::PutNumber("Vision Position Right", Robot::visionBridge->GetPosition(1));
	SmartDashboard::PutNumber("Vision Distance", Robot::visionBridge->GetDistance());
	*/
}

void Robot::TeleopInit() {
	driveTrain->enableSteeringPID(); //ENABLE THIS

	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// these lines or comment it out.
	if (autonomousCommand != nullptr) {
		autonomousCommand->Cancel();
	}
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
	SmartDashboard::PutBoolean("Indexer Jammed", indexer->IsIndexJammed());
	SmartDashboard::PutNumber("Index Timer", indexer->GetTimer());
	SmartDashboard::PutNumber("Reverse Time", indexer->GetReverseTime());
	//indexer->ReadPDP();
	driveTrain->Dashboard();
	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetAngle());
	/*
	SmartDashboard::PutNumber("Vision Position Left", Robot::visionBridge->GetPosition(0));
	SmartDashboard::PutNumber("Vision Position Right", Robot::visionBridge->GetPosition(1));
	SmartDashboard::PutNumber("Vision Distance", Robot::visionBridge->GetDistance());
	*/

	//SmartDashboard::PutNumber("Bottom Velocity", shooter->shooterMotor1->GetSpeed());
	//SmartDashboard::PutNumber("Top Velocity", shooter->shooterMotor2->GetSpeed());
	SmartDashboard::PutNumber("Target Shooter Speed", shooter->targetShooterSpeed);
	//SmartDashboard::PutNumber("Indexer Position", indexer->GetPosition());

	SmartDashboard::PutNumber("JoystickXAxis", oi->GetJoystickX2());

	/*
	SmartDashboard::PutNumber("JoystickYAxis", oi->GetJoystickY2());
	SmartDashboard::PutNumber("JoystickXAxis", oi->GetJoystickX2());

	SmartDashboard::PutBoolean("Button2", oi->GetButton2());
	SmartDashboard::PutBoolean("Button3", oi->GetButton3());
	SmartDashboard::PutBoolean("Button4", oi->GetButton4());
	SmartDashboard::PutBoolean("Button5", oi->GetButton5());
	SmartDashboard::PutBoolean("Trigger", oi->GetButtonTrig());

	SmartDashboard::PutBoolean("ButtonA", oi->GetButtonA());
*/
}

void Robot::TestPeriodic() {
	lw->Run();
}

void Robot::ScriptInit() {
	LOG("Robot::ScriptInit");

	CommandListParser &parser(CommandListParser::GetInstance());

	parser.AddCommand(CommandParseInfo(
			"Drive", {"D", "d"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto x = parameters[0];
		auto y = parameters[1];
		auto z = parameters[2];
		auto timeout = parameters[3];
		Command *command = new ScriptDrive("Drive", x, y, z, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"DriveTilVision", {"DV", "dv"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto y = parameters[0];
		auto x = parameters[1];
		auto z = parameters[2];
		auto timeout = parameters[3];
		Command *command = new DriveTilVision(y, x, z, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"WaitForVision", {"WAIT", "wait"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(0);
		Command *command = new WaitForVision();
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"BackupToCenter", {"BACK", "back"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(0);
		Command *command = new BackupToCenter();
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"Sleep", {"S", "s"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(1);
		auto timeout = parameters[0];
		Command *command = new ScriptSleep("Sleep", timeout);
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"DriveGyro", {"DG", "dg"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto x = parameters[0];
		auto y = parameters[1];
		auto desiredangle = parameters[2];
		auto timeout = parameters[3];
		Command *command = new ScriptGyroDrive("DriveGyro", x, y, desiredangle, timeout);
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"DriveDistance", {"DD", "dd"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto x = parameters[0];
		auto y = parameters[1];
		auto twist = parameters[2];
		auto distance = parameters[3];
		Command *command = new DriveDistance(x, y, twist, distance);
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"DriveCam", {"DC", "dc"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(5);
		auto x = parameters[0];
		auto y = parameters[1];
		auto maxspeed = parameters[2];
		auto timeout = parameters[3];
		auto preferredSide = parameters[4];
		Command *command = new ScriptCamDrive("DriveCam", x, y, maxspeed, timeout, preferredSide);
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"FieldCentricDrive", {"FC", "fc"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto y = parameters[0];
		auto x = parameters[1];
		auto z = parameters[2];
		auto timeout = parameters[3];
		Command *command = new ScriptFieldCentricCrab(z, y, x, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"ScriptShoot", {"SH", "sh"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		Command *command = new ScriptShoot();
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	// Call IsValid to ensure that regular expressions
	// get built before the start of autonomous.
	parser.IsValid("S(0)");
}

START_ROBOT_CLASS(Robot);
