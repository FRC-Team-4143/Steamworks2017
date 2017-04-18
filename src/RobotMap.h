#pragma once

#include <WPILib.h>
#include <AHRS.h>
#include <CANTalon.h>

class RobotMap {
public:
	static bool SpeedControl;

	static PowerDistributionPanel* pdp;

	static CANTalon* driveTrainFrontLeftDrive;
	static CANTalon* driveTrainFrontLeftSteer;

	static CANTalon* driveTrainFrontRightDrive;
	static CANTalon* driveTrainFrontRightSteer;

	static CANTalon* driveTrainRearLeftDrive;
	static CANTalon* driveTrainRearLeftSteer;

	static CANTalon* driveTrainRearRightDrive;
	static CANTalon* driveTrainRearRightSteer;

	static I2C* i2c;
	static I2C* lidar;

	static AnalogInput* sonar;

	static CANTalon* climbingMotor;
	static CANTalon* climbingMotor2;


	static AHRS* imu;

	//static SerialPort* serialPort;
	//static SerialPort* serialPort1;
	//static SerialPort* serialPort2;

    static CANTalon* shooterMotor1;
    static CANTalon* shooterMotor2;

	static CANTalon* pickup;
	static CANTalon* indexMotor;
	static CANTalon* turretMotor;

	static void Initialize();
};
