#pragma once

#include <WPILib.h>
#include <AHRS.h>
#include <CANTalon.h>
#include "Modules/AnalogChannelVolt.h"

class RobotMap {
public:
	static bool SpeedControl;

	static PIDController* driveTrainFrontLeft;
	static CANTalon* driveTrainFrontLeftDrive;
	static AnalogChannelVolt* driveTrainFrontLeftPos;
	static CANTalon* driveTrainFrontLeftSteer;

	static PIDController* driveTrainFrontRight;
	static CANTalon* driveTrainFrontRightDrive;
	static AnalogChannelVolt* driveTrainFrontRightPos;
	static CANTalon* driveTrainFrontRightSteer;

	static PIDController* driveTrainRearLeft;
	static CANTalon* driveTrainRearLeftDrive;
	static AnalogChannelVolt* driveTrainRearLeftPos;
	static CANTalon* driveTrainRearLeftSteer;

	static PIDController* driveTrainRearRight;
	static CANTalon* driveTrainRearRightDrive;
	static AnalogChannelVolt* driveTrainRearRightPos;
	static CANTalon* driveTrainRearRightSteer;

	static I2C* i2c;

	static AHRS* imu;

	static SerialPort* serialPort;

	static CANTalon* leftFront;
	static CANTalon* rightRear;
	static CANTalon* rightFront;
	static CANTalon* leftRear;

	static SpeedController* feeder;
	static AnalogInput* feederSensor;

	static CANTalon* armMotor;

	static CANTalon* winchMotor;
	static AnalogInput* winchPot;
	static PIDController* shooterWinch;

	static void Initialize();
};
