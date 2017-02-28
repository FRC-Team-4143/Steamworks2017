#include "Subsystems/DriveTrain.h"
#include "Commands/CrabDrive.h"
#include "Commands/PositionDrive.h"
#include "Modules/Constants.h"
#include "Modules/Logger.h"
#include "Robot.h"
#include "RobotMap.h"
#include "Subsystems/EncoderConstants.h"

#ifdef TESTSWERVE
#define MAXTURNS 3
#else
#define MAXTURNS 3000
#define SOFTTURNLIMIT 2
#endif

#define FLS 0
#define FRS 1
#define RLS 2
#define RRS 3

const float TWISTSCALE = 0.6;

const float DEAD_ZONE = 0.1;

const float driveScale = 950;
// const double AVERAGE_VOLTAGE_BASE = EncoderConstants::HALF_TURN;

#define GYROP .001
#define GYROMAX .5

// ==========================================================================

DriveTrain::DriveTrain() :
		frc::Subsystem("DriveTrain") {
	LOG("DriveTrain::ctor");

	unwinding = false;
	X = 0;
	Y = 0;

	FLOffset = 0;
	FROffset = 0;
	RLOffset = 0;
	RROffset = 0;

	FLInv = 1;
	FRInv = 1;
	RRInv = 1;
	RLInv = 1;

	lastx = 0.0;
	lasty = 0.0;
	lasttwist = 0.0;

	FLValue = 0;
	FRValue = 0;
	RLValue = 0;
	RRValue = 0;

	//frontLeft = RobotMap::driveTrainFrontLeft;
	frontLeftDrive = RobotMap::driveTrainFrontLeftDrive;
	//frontLeftPos = RobotMap::driveTrainFrontLeftPos;
	frontLeftSteer = RobotMap::driveTrainFrontLeftSteer;

	//frontRight = RobotMap::driveTrainFrontRight;
	frontRightDrive = RobotMap::driveTrainFrontRightDrive;
	//frontRightPos = RobotMap::driveTrainFrontRightPos;
	frontRightSteer = RobotMap::driveTrainFrontRightSteer;

	//rearLeft = RobotMap::driveTrainRearLeft;
	rearLeftDrive = RobotMap::driveTrainRearLeftDrive;
	//rearLeftPos = RobotMap::driveTrainRearLeftPos;
	rearLeftSteer = RobotMap::driveTrainRearLeftSteer;

	//rearRight = RobotMap::driveTrainRearRight;
	rearRightDrive = RobotMap::driveTrainRearRightDrive;
	//rearRightPos = RobotMap::driveTrainRearRightPos;
	rearRightSteer = RobotMap::driveTrainRearRightSteer;
}

// ==========================================================================

void DriveTrain::EnablePIDs(bool enable) {
	if (enable) {
		frontLeftSteer->Enable();
		frontRightSteer->Enable();
		rearLeftSteer->Enable();
		rearRightSteer->Enable();
	} else {
		frontLeftSteer->Disable();
		frontRightSteer->Disable();
		rearLeftSteer->Disable();
		rearRightSteer->Disable();
	}
}

// ==========================================================================

void DriveTrain::InitDefaultCommand() {
	SetDefaultCommand(new CrabDrive());
}

// ==========================================================================

void DriveTrain::SetWheelbase(float w, float x, float y) {
	//W = w;
	X = x;
	Y = y;
}

// ==========================================================================

void DriveTrain::SetOffsets(double FLOff, double FROff, double RLOff,
		double RROff) {
	FLOffset = FLOff;
	FROffset = FROff;
	RLOffset = RLOff;
	RROffset = RROff;
}

// ==========================================================================

bool DriveTrain::unwind() { // float y, float x){
	frontLeftSteer->Disable();
	frontRightSteer->Disable();
	rearLeftSteer->Disable();
	rearRightSteer->Disable();

	frontLeftSteer->SetControlMode(CANSpeedController::kPosition);
	frontRightSteer->SetControlMode(CANSpeedController::kPosition);
	rearLeftSteer->SetControlMode(CANSpeedController::kPosition);
	rearRightSteer->SetControlMode(CANSpeedController::kPosition);

	frontLeftSteer->SetP(0.4);
	frontRightSteer->SetP(0.4);
	rearLeftSteer->SetP(0.4);
	rearRightSteer->SetP(0.4);

	frontLeftSteer->ConfigPeakOutputVoltage(6.0, -6.0);
	frontRightSteer->ConfigPeakOutputVoltage(6.0, -6.0);
	rearLeftSteer->ConfigPeakOutputVoltage(6.0, -6.0);
	rearRightSteer->ConfigPeakOutputVoltage(6.0, -6.0);

	frontLeftSteer->Set(FLOffset / EncoderConstants::FULL_TURN);
	frontRightSteer->Set(FROffset / EncoderConstants::FULL_TURN);
	rearLeftSteer->Set(RLOffset / EncoderConstants::FULL_TURN);
	rearRightSteer->Set(RROffset / EncoderConstants::FULL_TURN);

	unwinding = true;
	return true;
}

// ==========================================================================

void DriveTrain::doneunwind() {
	unwinding = 0;
}

// ==========================================================================

void DriveTrain::PositionModeTwist(float desiredangle) {
	frontLeftSteer->SetSetpoint(CorrectSteerSetpoint(FLOffset + 0.625, frontLeftSteer));
	frontRightSteer->SetSetpoint(CorrectSteerSetpoint(FROffset - 0.625, frontRightSteer));
	rearLeftSteer->SetSetpoint(CorrectSteerSetpoint(RLOffset - 0.625, rearLeftSteer));
	rearRightSteer->SetSetpoint(CorrectSteerSetpoint(RROffset + 0.625, rearRightSteer));
}

// ==========================================================================
// keeps controls consistent regardless of rotation of robot

void DriveTrain::FieldCentricCrab(float twist, float y, float x,
		bool operatorControl) {
	float FWD = y;
	float STR = x;

	auto robotangle = Robot::gyroSub->PIDGet() * pi / 180;

	FWD = y * cos(robotangle) + x * sin(robotangle);
	STR = -y * sin(robotangle) + x * cos(robotangle);

	Crab(twist, FWD, STR, operatorControl);
}

// ==========================================================================
// attempts to keep robot square to the field as it drives

void DriveTrain::GyroCrab(float desiredangle, float y, float x,
		bool operatorControl) {
	auto robotangle = Robot::gyroSub->PIDGet();

	float twist = desiredangle - robotangle;
	while (twist > 180.0)
		twist -= 360.0;
	while (twist < -180.0)
		twist += 360.0;

	twist = std::min(GYROMAX, std::max(-GYROMAX, twist * GYROP));
	Crab(twist, y, x, operatorControl);
}

// ==========================================================================

void DriveTrain::Crab(float twist, float y, float x, bool operatorControl) {
	// stop PID loop if wires wrap.
	/*if (unwinding || abs(frontRightPos->GetTurns()) > MAXTURNS ||
	 abs(rearRightPos->GetTurns()) > MAXTURNS ||
	 abs(frontLeftPos->GetTurns()) > MAXTURNS ||
	 abs(rearLeftPos->GetTurns()) > MAXTURNS) {
	 SetDriveSpeed(0, 0, 0, 0);
	 return;
	 }
	 */
	// this stores the direction of joystick when all axis last crossed into the
	// deadzone and keeps the wheels pointed that direction
	// this .1 should be kept the same as the deadzone in oi.cpp
	if (operatorControl && x == 0.0 && y == 0.0 && twist == 0.0) {
		if (fabs(lasty) > DEAD_ZONE || fabs(lastx) > DEAD_ZONE
				|| fabs(lasttwist) > DEAD_ZONE) {
			y = std::min(std::max(lasty, -DEAD_ZONE), DEAD_ZONE);
			x = std::min(std::max(lastx, -DEAD_ZONE), DEAD_ZONE);
			twist = std::min(std::max(lasttwist, -DEAD_ZONE), DEAD_ZONE);
		} else {
			y = .05;
			// default wheel position
		}
	} else if (Robot::oi->GetButtonA() && !operatorControl && twist == 0.0) {
		twist = 0.05;
	}

	lastx = x;
	lasty = y;
	lasttwist = twist;

	if (operatorControl) {
		// scale for operator control
		x *= 1;
		y *= 1;
		float avg = (std::abs(x) + std::abs(y)) / 2;
		float scale = 1 - avg / 2;
		twist *= scale; // TWISTSCALE;
	}

	float FWD = y;
	float STR = x;

	auto radius = sqrt(pow(Y, 2) + pow(X, 2));

	auto AP = STR - twist * X / radius;
	auto BP = STR + twist * X / radius;
	auto CP = FWD - twist * Y / radius;
	auto DP = FWD + twist * Y / radius;

	float FLSetPoint = EncoderConstants::HALF_TURN;
	float FRSetPoint = EncoderConstants::HALF_TURN;
	float RLSetPoint = EncoderConstants::HALF_TURN;
	float RRSetPoint = EncoderConstants::HALF_TURN;

	if (DP != 0 || BP != 0) {
		FLSetPoint = (EncoderConstants::HALF_TURN
				+ EncoderConstants::HALF_TURN / pi * atan2(BP, DP));
	}
	if (BP != 0 || CP != 0) {
		FRSetPoint = (EncoderConstants::HALF_TURN
				+ EncoderConstants::HALF_TURN / pi * atan2(BP, CP));
	}
	if (AP != 0 || DP != 0) {
		RLSetPoint = (EncoderConstants::HALF_TURN
				+ EncoderConstants::HALF_TURN / pi * atan2(AP, DP));
	}
	if (AP != 0 || CP != 0) {
		RRSetPoint = (EncoderConstants::HALF_TURN
				+ EncoderConstants::HALF_TURN / pi * atan2(AP, CP));
	}

	SetSteer(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);

	double FL; // FL, distance from Front Left Wheel to the center of rotation
	double FR; // FR, distance from Front Right Wheel to the center of rotation
	double RL; // RL, distance from Rear Left Wheel to the center of rotation
	double RR; // RR, distance from Rear Right Wheel to the center of rotation

	FL = sqrt(pow(BP, 2) + pow(DP, 2));
	FR = sqrt(pow(BP, 2) + pow(CP, 2));
	RL = sqrt(pow(AP, 2) + pow(DP, 2));
	RR = sqrt(pow(AP, 2) + pow(CP, 2));

	// Solve for fastest wheel speed
	double speedarray[] = { fabs(FL), fabs(FR), fabs(RL), fabs(RR) };

	int length = 4;
	double maxspeed = speedarray[0];
	for (int i = 1; i < length; i++) {
		if (speedarray[i] > maxspeed) {
			maxspeed = speedarray[i];
		}
	}

	double FRRatio; // Ratio of Speed of Front Right wheel
	double FLRatio; // Ratio of Speed of Front Left wheel
	double RRRatio; // Ratio of Speed of Rear Right wheel
	double RLRatio; // Ratio of Speed of Rear Left wheel

	// Set ratios based on maximum wheel speed
	if (maxspeed > 1 || maxspeed < -1) {
		FLRatio = FL / maxspeed;
		FRRatio = FR / maxspeed;
		RLRatio = RL / maxspeed;
		RRRatio = RR / maxspeed;
	} else {
		FLRatio = FL;
		FRRatio = FR;
		RLRatio = RL;
		RRRatio = RR;
	}

	if (operatorControl && fabs(x) <= DEAD_ZONE && fabs(y) <= DEAD_ZONE
			&& fabs(twist) <= DEAD_ZONE) {
		FLRatio = 0.0;
		FRRatio = 0.0;
		RLRatio = 0.0;
		RRRatio = 0.0;
	}

	// Set drive speeds
	SetDriveSpeed(FLRatio, -FRRatio, RLRatio, -RRRatio);
}

// ==========================================================================

double DriveTrain::CorrectSteerSetpoint(double setpoint, CANTalon* talon) {
	if (setpoint < 0) {
		setpoint = (setpoint + EncoderConstants::FULL_TURN) / EncoderConstants::FULL_TURN;
	} else if (setpoint > EncoderConstants::FULL_TURN) {
		setpoint = (setpoint - EncoderConstants::FULL_TURN) / EncoderConstants::FULL_TURN;
	}

	float currentPosition = talon->GetPosition();

	if (setpoint < 0.25 && currentPosition - trunc(currentPosition) > 0.75) {
		return (trunc(currentPosition) + setpoint + 1);
	} else if (setpoint > 0.75 && currentPosition - trunc(currentPosition) < 0.25) {
		return (trunc(currentPosition) + setpoint - 1);
	} else {
		return trunc(currentPosition) + setpoint;
	}
	/*
	 int newSetpoint = (trunc(currentPosition)+setpoint);

	 while (newSetpoint - currentPosition < -EncoderConstants::FULL_TURN){
	 newSetpoint -= EncoderConstants::FULL_TURN;
	 }
	 while (newSetpoint - currentPosition > EncoderConstants::FULL_TURN){
	 newSetpoint += EncoderConstants::FULL_TURN;
	 }

	 return newSetpoint / EncoderConstants::FULL_TURN;


	 if (setpoint < 0) {
	 return (setpoint + EncoderConstants::FULL_TURN)/EncoderConstants::FULL_TURN;
	 }
	 else if (setpoint > EncoderConstants::FULL_TURN) {
	 return (setpoint - EncoderConstants::FULL_TURN)/EncoderConstants::FULL_TURN;
	 }
	 else if (setpoint == EncoderConstants::FULL_TURN) {
	 return 0;
	 }
	 else {
	 return setpoint/5;
	 }*/
}

// ==========================================================================

//REMINDER: add the offset before this function
void setSteerSetpoint(float setpoint, CANTalon* talon, double *inverse){
	float currentPosition = talon->GetPosition();
	int turns = trunc(currentPosition);
	int currentAngle = currentPosition - turns;

	currentPosition *= EncoderConstants::FULL_TURN;
	turns *= EncoderConstants::FULL_TURN;
	currentAngle *= EncoderConstants::FULL_TURN;

	float angleOptions[6];
	angleOptions[0] = turns - EncoderConstants::FULL_TURN + setpoint;
	angleOptions[1] = turns - EncoderConstants::FULL_TURN + setpoint + EncoderConstants::HALF_TURN;
	angleOptions[2] = turns + setpoint;
	angleOptions[3] = turns + setpoint + EncoderConstants::HALF_TURN;
	angleOptions[4] = turns + EncoderConstants::FULL_TURN + setpoint;
	angleOptions[5] = turns + EncoderConstants::FULL_TURN + setpoint + EncoderConstants::HALF_TURN;

	float minMove = fabs(currentPosition - angleOptions[0]);
	int minI = 0;
	for (int i = 1; i < 6; i++){
		if (fabs(currentPosition - angleOptions[i]) < minMove){
			minMove = fabs(currentPosition - angleOptions[i]);
			minI = i;
		}
	}
	talon->SetSetpoint(angleOptions[minI]/EncoderConstants::FULL_TURN);

	*inverse = cos((minMove / EncoderConstants::FULL_TURN) * 2 * 3.141);
	if (minI % 2)
		*inverse *= -1;
	else
		*inverse *= 1;

}

void DriveTrain::SetSteer(float FLSetPoint, float FRSetPoint,
		float RLSetPoint, float RRSetPoint) {
	FLSetPoint = -FLSetPoint;
	FRSetPoint = -FRSetPoint;
	RLSetPoint = -RLSetPoint;
	RRSetPoint = -RRSetPoint;

	setSteerSetpoint(FLSetPoint + FLOffset, frontLeftSteer, &FLInv);
	setSteerSetpoint(FRSetPoint + FROffset, frontRightSteer, &FRInv);
	setSteerSetpoint(RLSetPoint + RLOffset, rearLeftSteer, &RLInv);
	setSteerSetpoint(RRSetPoint + RROffset, rearRightSteer, &RRInv);

	return;

	//////////////////////////////////
	// Front Left Wheel
	//////////////////////////////////
	/*	if (frontLeftPos->GetTurns() > SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(FLSetPoint + FLOffset - frontLeftPos->GetAngle()) > EncoderConstants::HALF_TURN) {
	 frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset));
	 FLInv = 1;
	 }
	 else {
	 frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset - EncoderConstants::HALF_TURN));
	 FLInv = -1;
	 }
	 }
	 else if (frontLeftPos->GetTurns() < -SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(FLSetPoint + FLOffset - frontLeftPos->GetAngle()) < EncoderConstants::HALF_TURN) {
	 frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset));
	 FLInv = 1;
	 }
	 else {
	 frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset - EncoderConstants::HALF_TURN));
	 FLInv = -1;
	 }
	 }
	 else {*/
	// Default rotation logic
	if (fabs(FLSetPoint + FLOffset - getTalonPosition(frontLeftSteer))
			< EncoderConstants::QUARTER_TURN
			|| fabs(FLSetPoint + FLOffset - getTalonPosition(frontLeftSteer))
					> EncoderConstants::THREEQUARTER_TURN) {
		frontLeftSteer->SetSetpoint(
				CorrectSteerSetpoint(FLSetPoint + FLOffset, frontLeftSteer));
		FLInv = 1;
	} else {
		frontLeftSteer->SetSetpoint(
				CorrectSteerSetpoint(
						FLSetPoint + FLOffset - EncoderConstants::HALF_TURN,
						frontLeftSteer));
		FLInv = -1;
	}
	//}

	//////////////////////////////////
	// Front Right Wheel
	//////////////////////////////////
	/*if (frontRightPos->GetTurns() > SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(FRSetPoint + FROffset - frontRightPos->GetAngle()) > EncoderConstants::HALF_TURN) {
	 frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset));
	 FRInv = 1;
	 }
	 else {
	 frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset - EncoderConstants::HALF_TURN));
	 FRInv = -1;
	 }
	 }
	 else if (frontRightPos->GetTurns() < -SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(FRSetPoint + FROffset - frontRightPos->GetAngle()) < EncoderConstants::HALF_TURN) {
	 frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset));
	 FRInv = 1;
	 }
	 else {
	 frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset - EncoderConstants::HALF_TURN));
	 FRInv = -1;
	 }
	 }
	 else {*/
	// default rotation logic
	if (fabs(FRSetPoint + FROffset - getTalonPosition(frontRightSteer))
			< EncoderConstants::QUARTER_TURN
			|| fabs(FRSetPoint + FROffset - getTalonPosition(frontRightSteer))
					> EncoderConstants::THREEQUARTER_TURN) {
		frontRightSteer->SetSetpoint(
				CorrectSteerSetpoint(FRSetPoint + FROffset, frontRightSteer));
		FRInv = 1;
	} else {
		frontRightSteer->SetSetpoint(
				CorrectSteerSetpoint(
						FRSetPoint + FROffset - EncoderConstants::HALF_TURN,
						frontRightSteer));
		FRInv = -1;
	}
	//}

	//////////////////////////////////
	// Rear Left Wheel
	//////////////////////////////////
	/*if (rearLeftPos->GetTurns() > SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(RLSetPoint + RLOffset - rearLeftPos->GetAngle()) > EncoderConstants::HALF_TURN) {
	 rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset));
	 RLInv = 1;
	 }
	 else {
	 rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset - EncoderConstants::HALF_TURN));
	 RLInv = -1;
	 }
	 }
	 else if (rearLeftPos->GetTurns() < -SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(RLSetPoint + RLOffset - rearLeftPos->GetAngle()) < EncoderConstants::HALF_TURN) {
	 rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset));
	 RLInv = 1;
	 }
	 else {
	 rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset - EncoderConstants::HALF_TURN));
	 RLInv = -1;
	 }
	 }
	 else {*/
	// default rotation logic
	if (fabs(RLSetPoint + RLOffset - getTalonPosition(rearLeftSteer))
			< EncoderConstants::QUARTER_TURN
			|| fabs(RLSetPoint + RLOffset - getTalonPosition(rearLeftSteer))
					> EncoderConstants::THREEQUARTER_TURN) {
		rearLeftSteer->SetSetpoint(
				CorrectSteerSetpoint(RLSetPoint + RLOffset, rearLeftSteer));
		RLInv = 1;
	} else {
		rearLeftSteer->SetSetpoint(
				CorrectSteerSetpoint(
						RLSetPoint + RLOffset - EncoderConstants::HALF_TURN,
						rearLeftSteer));
		RLInv = -1;
	}
	//}

	//////////////////////////////////
	// Rear Right Wheel
	//////////////////////////////////
	/*if (rearRightPos->GetTurns() > SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(RRSetPoint + RROffset - rearRightPos->GetAngle()) > EncoderConstants::HALF_TURN) {
	 rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset));
	 RRInv = 1;
	 }
	 else {
	 rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset - EncoderConstants::HALF_TURN));
	 RRInv = -1;
	 }
	 }
	 else if (rearRightPos->GetTurns() < -SOFTTURNLIMIT) {
	 if (CorrectSteerSetpoint(RRSetPoint + RROffset - rearRightPos->GetAngle()) < EncoderConstants::HALF_TURN) {
	 rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset));
	 RRInv = 1;
	 }
	 else {
	 rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset - EncoderConstants::HALF_TURN));
	 RRInv = -1;
	 }
	 }
	 else {*/
	// default rotation logic
	if (fabs(RRSetPoint + RROffset - getTalonPosition(rearRightSteer))
			< EncoderConstants::QUARTER_TURN
			|| fabs(RRSetPoint + RROffset - getTalonPosition(rearRightSteer))
					> EncoderConstants::THREEQUARTER_TURN) {
		rearRightSteer->SetSetpoint(
				CorrectSteerSetpoint(RRSetPoint + RROffset, rearRightSteer));
		RRInv = 1;
	} else {
		rearRightSteer->SetSetpoint(
				CorrectSteerSetpoint(
						RRSetPoint + RROffset - EncoderConstants::HALF_TURN,
						rearRightSteer));
		RRInv = -1;
	}
	//}
}

// ==========================================================================

void DriveTrain::SetDriveSpeed(float FLSpeed, float FRSpeed, float RLSpeed,
		float RRSpeed) {
	if (RobotMap::SpeedControl) {
		frontLeftDrive->Set(FLSpeed * FLInv * driveScale);
		frontRightDrive->Set(FRSpeed * FRInv * driveScale);
		rearLeftDrive->Set(RLSpeed * RLInv * driveScale);
		rearRightDrive->Set(RRSpeed * RRInv * driveScale);
	} else {
		frontLeftDrive->Set(FLSpeed * FLInv);
		frontRightDrive->Set(FRSpeed * FRInv);
		rearLeftDrive->Set(RLSpeed * RLInv);
		rearRightDrive->Set(RRSpeed * RRInv);
	}
}

// ==========================================================================

void DriveTrain::Lock() {
	SetSteer(3.0, 1.5, 3.0, 1.5);
	SetDriveSpeed(0, 0, 0, 0);
}

// ==========================================================================

void DriveTrain::SideLock() {
	SetSteer(2.0, 0.75, 3.25, 4.5);
	SetDriveSpeed(0, 0, 0, 0);
}

// ==========================================================================

void DriveTrain::updateDistanceEncoders() {
	FLValue = frontLeftDrive->GetPosition();
	FRValue = frontRightDrive->GetPosition();
	RLValue = rearLeftDrive->GetPosition();
	RRValue = rearRightDrive->GetPosition();
}

// ==========================================================================

double DriveTrain::getDistanceEncodersValues() {
	double average = (FLValue + FRValue + RLValue + RRValue) / 4;
	return average;
}

// ==========================================================================

void DriveTrain::zeroDistanceEncoders() {
	frontLeftDrive->SetPosition(0);
	frontRightDrive->SetPosition(0);
	rearLeftDrive->SetPosition(0);
	rearRightDrive->SetPosition(0);
}

// ==========================================================================

void DriveTrain::zeroSteeringEncoders() {
	frontLeftSteer->SetPosition(0);
	frontRightSteer->SetPosition(0);
	rearLeftSteer->SetPosition(0);
	rearRightSteer->SetPosition(0);
}

// ==========================================================================

void DriveTrain::setWheelOffsets() {
	// Get the current steering positions from the drive train.
	/*
	 auto FLPosition = Robot::driveTrain->frontLeftPos->GetRawAngle();
	 auto FRPosition = Robot::driveTrain->frontRightPos->GetRawAngle();
	 auto RLPosition = Robot::driveTrain->rearLeftPos->GetRawAngle();
	 auto RRPosition = Robot::driveTrain->rearRightPos->GetRawAngle();
	 auto ArmPosition = Robot::armSub->GetRawPosition();
	 */
	auto FLPosition = getTalonPosition(frontLeftSteer);
	auto FRPosition = getTalonPosition(frontRightSteer);
	auto RLPosition = getTalonPosition(rearLeftSteer);
	auto RRPosition = getTalonPosition(rearRightSteer);
	LogSettings(FLPosition, FRPosition, RLPosition, RRPosition);

	// Save the positions to Preferences.
	auto prefs = Preferences::GetInstance();
	prefs->PutDouble(Constants::FL_POS_NAME, FLPosition);
	prefs->PutDouble(Constants::FR_POS_NAME, FRPosition);
	prefs->PutDouble(Constants::RL_POS_NAME, RLPosition);
	prefs->PutDouble(Constants::RR_POS_NAME, RRPosition);
	//prefs->PutDouble(Constants::ARM_POSITION_NAME, ArmPosition);

	// Set the drive train positions.
	SetOffsets(FLPosition, FRPosition, RLPosition, RRPosition);
	//Robot::armSub->SetOffset(ArmPosition);
}

// ==========================================================================

void DriveTrain::loadWheelOffsets() {
	LOG("DriveTrainSettings::LoadSettings");

	// Load the positions from Preferences.
	auto prefs = Preferences::GetInstance();
	auto FLPosition = prefs->GetDouble(Constants::FL_POS_NAME,
			Constants::FL_POS_DEFAULT);
	auto FRPosition = prefs->GetDouble(Constants::FR_POS_NAME,
			Constants::FR_POS_DEFAULT);
	auto RLPosition = prefs->GetDouble(Constants::RL_POS_NAME,
			Constants::RL_POS_DEFAULT);
	auto RRPosition = prefs->GetDouble(Constants::RR_POS_NAME,
			Constants::RR_POS_DEFAULT);

	LogSettings(FLPosition, FRPosition, RLPosition, RRPosition);

	// Set the drive train positions.
	SetOffsets(FLPosition, FRPosition, RLPosition, RRPosition);
}

// ==========================================================================

void DriveTrain::LogSettings(double fl, double fr, double rl, double rr) {
	char sz[256];
	sprintf(sz, "Wheel Positions: FL %f, FR %f, RL %f, RR %f", fl, fr, rl, rr);
	LOG(sz);
}

// ==========================================================================

void DriveTrain::Dashboard() {
	SmartDashboard::PutNumber("Steering Motor Encoder FL",
			frontLeftSteer->GetPosition());
	SmartDashboard::PutNumber("Steering Motor Encoder FR",
			frontRightSteer->GetPosition());
	SmartDashboard::PutNumber("Steering Motor Encoder RL",
			rearLeftSteer->GetPosition());
	SmartDashboard::PutNumber("Steering Motor Encoder RR",
			rearRightSteer->GetPosition());

	SmartDashboard::PutNumber("FR Setpoint", frontRightSteer->GetSetpoint());
	SmartDashboard::PutNumber("FL Setpoint", frontLeftSteer->GetSetpoint());
	SmartDashboard::PutNumber("RR Setpoint", rearRightSteer->GetSetpoint());
	SmartDashboard::PutNumber("RL Setpoint", rearLeftSteer->GetSetpoint());
}

// ==========================================================================

void DriveTrain::CrabInit() {
	frontLeftSteer->Enable();
	frontRightSteer->Enable();
	rearLeftSteer->Enable();
	rearRightSteer->Enable();
	frontLeftSteer->SetControlMode(CANSpeedController::kPosition);
	frontRightSteer->SetControlMode(CANSpeedController::kPosition);
	rearLeftSteer->SetControlMode(CANSpeedController::kPosition);
	rearRightSteer->SetControlMode(CANSpeedController::kPosition);
}

// ==========================================================================

void DriveTrain::SetWheelsStraight() {
	frontLeftSteer->SetSetpoint(CorrectSteerSetpoint(FLOffset, frontLeftSteer));
	frontRightSteer->SetSetpoint(CorrectSteerSetpoint(FROffset, frontRightSteer));
	rearLeftSteer->SetSetpoint(CorrectSteerSetpoint(RLOffset, rearLeftSteer));
	rearRightSteer->SetSetpoint(CorrectSteerSetpoint(RROffset, rearRightSteer));
}

// ==========================================================================

void DriveTrain::ArcadeDriveMode(float x, float y) {
	float leftMotorOutput;
	float rightMotorOutput;

	if (y > 0.0) {
		if (x > 0.0) {
			leftMotorOutput = y - x;
			rightMotorOutput = std::max(y, x);
		} else {
			leftMotorOutput = std::max(y, -x);
			rightMotorOutput = y + x;
		}
	} else {
		if (x > 0.0) {
			leftMotorOutput = -std::max(-y, x);
			rightMotorOutput = y + x;
		} else {
			leftMotorOutput = y - x;
			rightMotorOutput = -std::max(-y, -x);
		}
	}

	SetDriveSpeed(leftMotorOutput, rightMotorOutput, leftMotorOutput,
			rightMotorOutput);
}

// ==========================================================================

void DriveTrain::disableSpeedControl() {
	frontLeftDrive->SetControlMode(CANSpeedController::kPercentVbus);
	frontRightDrive->SetControlMode(CANSpeedController::kPercentVbus);
	rearLeftDrive->SetControlMode(CANSpeedController::kPercentVbus);
	rearRightDrive->SetControlMode(CANSpeedController::kPercentVbus);
	RobotMap::SpeedControl = 0;
}

// ==========================================================================

void DriveTrain::enableSpeedControl() {
	frontLeftDrive->SelectProfileSlot(0);
	frontRightDrive->SelectProfileSlot(0);
	rearLeftDrive->SelectProfileSlot(0);
	rearRightDrive->SelectProfileSlot(0);
	frontLeftDrive->ConfigPeakOutputVoltage(12.0, -12.0);
	frontRightDrive->ConfigPeakOutputVoltage(12.0, -12.0);
	rearLeftDrive->ConfigPeakOutputVoltage(12.0, -12.0);
	rearRightDrive->ConfigPeakOutputVoltage(12.0, -12.0);
	frontLeftDrive->SetControlMode(CANSpeedController::kSpeed);
	frontRightDrive->SetControlMode(CANSpeedController::kSpeed);
	rearLeftDrive->SetControlMode(CANSpeedController::kSpeed);
	rearRightDrive->SetControlMode(CANSpeedController::kSpeed);
	RobotMap::SpeedControl = 1;
}

// ==========================================================================

void DriveTrain::enablePositionControl() {
	frontLeftDrive->SelectProfileSlot(1);
	frontRightDrive->SelectProfileSlot(1);
	rearLeftDrive->SelectProfileSlot(1);
	rearRightDrive->SelectProfileSlot(1);
	frontLeftDrive->ConfigPeakOutputVoltage(4.0, -4.0);
	frontRightDrive->ConfigPeakOutputVoltage(4.0, -4.0);
	rearLeftDrive->ConfigPeakOutputVoltage(4.0, -4.0);
	rearRightDrive->ConfigPeakOutputVoltage(4.0, -4.0);
	frontLeftDrive->SetControlMode(CANSpeedController::kPosition);
	frontRightDrive->SetControlMode(CANSpeedController::kPosition);
	rearLeftDrive->SetControlMode(CANSpeedController::kPosition);
	rearRightDrive->SetControlMode(CANSpeedController::kPosition);
	RobotMap::SpeedControl = 2;
}

// ==========================================================================

void DriveTrain::enableSteeringPID() {
	frontLeftSteer->Enable();
	frontRightSteer->Enable();
	rearLeftSteer->Enable();
	rearRightSteer->Enable();
}

// ==========================================================================

double DriveTrain::getTalonPosition(CANTalon* talon) {
	return (talon->GetPosition() - trunc(talon->GetPosition()))*EncoderConstants::FULL_TURN;
}
