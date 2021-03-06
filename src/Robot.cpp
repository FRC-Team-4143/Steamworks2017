#include "Robot.h"
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
#include "Commands/ScriptShootWithVision.h"
#include "Commands/DriveTilLidar.h"
#include "Commands/DriveTilSonar.h"
#include "Commands/ScriptGyroRotate.h"
#include "Commands/GearLineup.h"
#include "Commands/GearLineupAndSonar.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"
#include "Modules/ScriptCommandFactory.h"
#include "Commands/BoilerLineup.h"

OI* Robot::oi;
Pickup* Robot::pickup = nullptr;
Indexer* Robot::indexer = nullptr;
Turret* Robot::turret = nullptr;
GyroSub* Robot::gyroSub = nullptr;
DriveTrain* Robot::driveTrain = nullptr;
std::shared_ptr<BasicCameraSub> Robot::basicCameraSub;
VisionBridgeSub* Robot::visionBridge = nullptr;
Shooter* Robot::shooter = nullptr;
Climber* Robot::climber = nullptr;
Servo* Robot::servo = nullptr;
Servo* Robot::servo2 = nullptr;

void Robot::RobotInit() {
	Preferences::GetInstance();

	RobotMap::Initialize();

	ScriptInit();
	SmartDashboard::PutString("ScriptCommand", "S(0.5)");
	SmartDashboard::PutString("ScriptValid", "");
	SmartDashboard::PutNumber("Twist Angle", 0);
	SmartDashboard::PutNumber("Servo Setpoint", 0);

	SmartDashboard::PutNumber("Serial 0", 1);
	SmartDashboard::PutNumber("Serial 1", 1);
	SmartDashboard::PutNumber("Serial 2", 1);

	printf("*** Initializing subsystems\r\n");
	printf("--- Initializing GyroSub\r\n");
	gyroSub = new GyroSub();
	printf("--- Initializing DriveTrain\r\n");
	driveTrain = new DriveTrain();
	printf("--- Initializing Pickup\r\n");
	pickup = new Pickup();
	printf("--- Initializing Indexer\r\n");
	indexer = new Indexer();
	printf("--- Initializing Turret\r\n");
	turret = new Turret();
	printf("--- Initializing BasicCameraSub\r\n");
	basicCameraSub.reset(new BasicCameraSub("cam0"));
	printf("--- Initializing VisionBridgeSub\r\n");
	visionBridge = new VisionBridgeSub();
	printf("--- Initializing Climber\r\n");
	climber = new Climber();

	printf("--- Initializing Shooter\r\n");
	shooter = new Shooter();

	printf("--- Setting wheelbase\r\n");
	driveTrain->SetWheelbase(26.8, 26.8, 22.8);
	printf("--- Loading wheel offsets\r\n");
	driveTrain->loadWheelOffsets();

	printf("*** Initializing OI\r\n");
	oi = new OI();

	printf("*** Initializing servos\r\n");
	servo = new Servo(0);
	servo2 = new Servo(1);
}

void Robot::RobotPeriodic() {
}

void Robot::DisabledInit() {
	//char mode = 1;
	//RobotMap::serialPort->Write(&mode, 1);
	//RobotMap::serialPort1->Write(&mode, 1);
	//RobotMap::serialPort2->Write(&mode, 1);
	//SmartDashboard::PutNumber("Serial 0", 1);
	//SmartDashboard::PutNumber("Serial 1", 1);
	//SmartDashboard::PutNumber("Serial 2", 1);
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
	//driveTrain->Dashboard();
	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetYaw());
	//indexer->ReadPDP();

	//SmartDashboard::PutNumber("Sonar", RobotMap::sonar->GetAverageVoltage());

	//char mode = SmartDashboard::GetNumber("Serial 0", 1);
	//RobotMap::serialPort->Write(&mode, 1);
	//char mode1 = SmartDashboard::GetNumber("Serial 1", 1);
	//RobotMap::serialPort1->Write(&mode1, 1);
	//char mode2 = SmartDashboard::GetNumber("Serial 2", 1);
	//RobotMap::serialPort2->Write(&mode2, 1);

	//SmartDashboard::PutNumber("Vision Gear Position", Robot::visionBridge->GetGearPosition());
	//SmartDashboard::PutNumber("Vision Gear Distance", Robot::visionBridge->GetGearDistance());
	//SmartDashboard::PutNumber("Vision Boiler Position", Robot::visionBridge->GetBoilerPosition());
	//SmartDashboard::PutNumber("Vision BoilerDistance", Robot::visionBridge->GetBoilerDistance());

	//SmartDashboard::PutNumber("Turret Starting Position", turret->startingPosition);
	//SmartDashboard::PutNumber("TurretPosition", RobotMap::turretMotor->GetSelectedSensorPosition(0)-turret->startingPosition);

	driveTrain->readLidar();

	//SmartDashboard::PutNumber("Bottom Velocity", shooter->shooterMotor1->GetSpeed());
	//SmartDashboard::PutNumber("Top Velocity", shooter->shooterMotor2->GetSpeed());
	SmartDashboard::PutNumber("Target Shooter Speed", shooter->targetShooterSpeed);
	//SmartDashboard::PutNumber("Indexer Position", indexer->GetPosition());
}

void Robot::AutonomousInit() {
	printf("Match time start: %f\r\n", DriverStation::GetInstance().GetMatchTime());
	driveTrain->enableSteeringPID();
	RobotMap::imu->ZeroYaw();

	turret->startingPosition = RobotMap::turretMotor->GetSelectedSensorPosition(0);

	//char mode = 3;
	//RobotMap::serialPort->Write(&mode, 1);
	//RobotMap::serialPort1->Write(&mode, 1);
	//RobotMap::serialPort2->Write(&mode, 1);
	//SmartDashboard::PutNumber("Serial 0", 3);
	//SmartDashboard::PutNumber("Serial 1", 3);
	//SmartDashboard::PutNumber("Serial 2", 3);

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

	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetYaw());
	SmartDashboard::PutNumber("Sonar", RobotMap::sonar->GetAverageVoltage());

	SmartDashboard::PutNumber("Vision Gear Position", Robot::visionBridge->GetGearPosition());
	SmartDashboard::PutNumber("Vision Gear Distance", Robot::visionBridge->GetGearDistance());
	SmartDashboard::PutNumber("Vision Boiler Position", Robot::visionBridge->GetBoilerPosition());
	SmartDashboard::PutNumber("Vision BoilerDistance", Robot::visionBridge->GetBoilerDistance());

	SmartDashboard::PutNumber("Turret Starting Position", turret->startingPosition);
	SmartDashboard::PutNumber("TurretPosition", RobotMap::turretMotor->GetSelectedSensorPosition(0)-turret->startingPosition);

	driveTrain->readLidar();
}

void Robot::TeleopInit() {
	driveTrain->enableSteeringPID();

	//char mode = 3;
	//RobotMap::serialPort->Write(&mode, 1);
	//RobotMap::serialPort1->Write(&mode, 1);
	//RobotMap::serialPort2->Write(&mode, 1);
	//SmartDashboard::PutNumber("Serial 0", 3);
	//SmartDashboard::PutNumber("Serial 1", 3);
	//SmartDashboard::PutNumber("Serial 2", 3);

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
	//SmartDashboard::PutBoolean("Indexer Jammed", indexer->IsIndexJammed());
	//SmartDashboard::PutNumber("Index Timer", indexer->GetTimer());
	//SmartDashboard::PutNumber("Reverse Time", indexer->GetReverseTime());
	//indexer->ReadPDP();
	//driveTrain->Dashboard();
	SmartDashboard::PutNumber("Gyro Yaw", RobotMap::imu->GetYaw());
	//SmartDashboard::PutNumber("Sonar", RobotMap::sonar->GetAverageVoltage());

	//SmartDashboard::PutNumber("Turret Starting Position", turret->startingPosition);
	//SmartDashboard::PutNumber("TurretPosition", RobotMap::turretMotor->GetSelectedSensorPosition(0)-turret->startingPosition);

	//SmartDashboard::PutNumber("Vision Gear Position", Robot::visionBridge->GetGearPosition());
	//SmartDashboard::PutNumber("Vision Gear Distance", Robot::visionBridge->GetGearDistance());
	//SmartDashboard::PutNumber("Vision Boiler Position", Robot::visionBridge->GetBoilerPosition());
	//SmartDashboard::PutNumber("Vision BoilerDistance", Robot::visionBridge->GetBoilerDistance());

	//char mode = SmartDashboard::GetNumber("Serial 0", 3);
	//RobotMap::serialPort->Write(&mode, 1);
	//char mode1 = SmartDashboard::GetNumber("Serial 1", 3);
	//RobotMap::serialPort1->Write(&mode1, 1);
	//char mode2 = SmartDashboard::GetNumber("Serial 2", 3);
	//RobotMap::serialPort2->Write(&mode2, 1);

	driveTrain->readLidar();

	/*make controller rumble at 30 seconds left
	if (ds.GetMatchTime() < 30 && ds.GetMatchTime() > 25) {
		Robot::oi->GetDriverJoystick()->SetRumble(Joystick::kLeftRumble, 1);
		Robot::oi->GetDriverJoystick()->SetRumble(Joystick::kRightRumble, 1);
	} else {
		Robot::oi->GetDriverJoystick()->SetRumble(Joystick::kLeftRumble, 0);
		Robot::oi->GetDriverJoystick()->SetRumble(Joystick::kRightRumble, 0);
	}
	*/
	//SmartDashboard::PutNumber("Bottom Velocity", shooter->shooterMotor1->GetSelectedSensorVelocity, 0);
	//SmartDashboard::PutNumber("Top Velocity", shooter->shooterMotor2->GetSelectedSensorVelocity, 0);
	SmartDashboard::PutNumber("Target Shooter Speed", shooter->targetShooterSpeed);
	//SmartDashboard::PutNumber("Indexer Position", indexer->GetPosition());

	SmartDashboard::PutNumber("JoystickXAxis", oi->GetJoystickX2());
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
		auto y = parameters[0];
		auto x = parameters[1];
		auto z = parameters[2];
		auto timeout = parameters[3];
		Command *command = new ScriptDrive("Drive", y, x, z, timeout);
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
		auto y = parameters[0];
		auto x = parameters[1];
		auto desiredangle = parameters[2];
		auto timeout = parameters[3];
		Command *command = new ScriptGyroDrive("DriveGyro", y, x, desiredangle, timeout);
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"GyroRotate", {"GR", "gr"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(3);
		auto power = parameters[0];
		auto desiredAngle = parameters[1];
		auto timeout = parameters[2];
		Command *command = new ScriptGyroRotate("GyroRotate", desiredAngle, power, timeout);
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
		parameters.resize(2);
		auto speed = parameters[0];
		auto timeout = parameters[1];
		Command *command = new ScriptShoot(speed, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"ScriptShootWithVision", {"VSH", "vsh"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(2);
		auto speed = parameters[0];
		auto timeout = parameters[1];
		Command *command;
		if (speed == -1)
			command = new ScriptShootWithVision(timeout);
		else
			command = new ScriptShootWithVision(speed, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"DriveTilLidar", {"DL", "dl"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(5);
		auto target = parameters[0];
		auto y = parameters[1];
		auto x = parameters[2];
		auto z = parameters[3];
		auto timeout = parameters[4];
		Command *command = new DriveTilLidar(target, y, x, z, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"DriveTilSonar", {"DS", "ds"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto y = parameters[0];
		auto x = parameters[1];
		auto z = parameters[2];
		auto timeout = parameters[3];
		Command *command = new DriveTilSonar(y, x, z, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"GearLineup", {"GL", "gl"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(0);
		Command *command = new GearLineup();
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
			"GearLineupAndSonar", {"GLS", "gls"},
			[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
		parameters.resize(4);
		auto y = parameters[0];
		auto x = parameters[1];
		auto z = parameters[2];
		auto timeout = parameters[3];
		Command *command = new GearLineupAndSonar(y, x, z, timeout);
		// if (0 == timeout) timeout = 4;
		fCreateCommand(command, 0);
	}));

	parser.AddCommand(CommandParseInfo(
				"BoilerLineup", {"BL", "bl"},
				[](std::vector<float> parameters, std::function<void(Command *, float)> fCreateCommand) {
			parameters.resize(0);
			Command *command = new BoilerLineup(0, 0);
			// if (0 == timeout) timeout = 4;
			fCreateCommand(command, 0);
		}));

	// Call IsValid to ensure that regular expressions
	// get built before the start of autonomous.
	parser.IsValid("S(0)");
}

START_ROBOT_CLASS(Robot);
