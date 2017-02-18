#pragma once

#include <WPILib.h>
#include <AHRS.h>
#include <CANTalon.h>
#include "Modules/AnalogChannelVolt.h"

class RobotMap {
public:
	static bool SpeedControl;

	static PowerDistributionPanel* pdp;

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

    //static CANTalon *shooterLeftFront;
    //static CANTalon *shooterLeftBack;
    //static CANTalon *shooterRightFront;
    //static CANTalon *shooterRightBack;

    static CANTalon* shooterMotor1;
    static CANTalon* shooterMotor2;

	static SpeedController* feeder;
	static AnalogInput* feederSensor;

	static CANTalon* armMotor;

	static CANTalon* pickup;
	static CANTalon* indexMotor;
	static CANTalon* winchMotor;
	static AnalogInput* winchPot;
	static PIDController* shooterWinch;

	static void Initialize();
};
