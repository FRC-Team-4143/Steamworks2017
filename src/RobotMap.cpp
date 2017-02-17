#include "RobotMap.h"
#include <LiveWindow/LiveWindow.h>

bool RobotMap::SpeedControl = 0;

PIDController*     RobotMap::driveTrainFrontLeft = nullptr;
CANTalon*          RobotMap::driveTrainFrontLeftDrive = nullptr;
AnalogChannelVolt* RobotMap::driveTrainFrontLeftPos = nullptr;
CANTalon*          RobotMap::driveTrainFrontLeftSteer = nullptr;

PIDController*     RobotMap::driveTrainFrontRight = nullptr;
CANTalon*          RobotMap::driveTrainFrontRightDrive = nullptr;
AnalogChannelVolt* RobotMap::driveTrainFrontRightPos = nullptr;
CANTalon*          RobotMap::driveTrainFrontRightSteer = nullptr;

PIDController*     RobotMap::driveTrainRearLeft = nullptr;
CANTalon*          RobotMap::driveTrainRearLeftDrive = nullptr;
AnalogChannelVolt* RobotMap::driveTrainRearLeftPos = nullptr;
CANTalon*          RobotMap::driveTrainRearLeftSteer = nullptr;

PIDController*     RobotMap::driveTrainRearRight = nullptr;
CANTalon*          RobotMap::driveTrainRearRightDrive = nullptr;
AnalogChannelVolt* RobotMap::driveTrainRearRightPos = nullptr;
CANTalon*          RobotMap::driveTrainRearRightSteer = nullptr;

I2C* RobotMap::i2c = nullptr;

SerialPort* RobotMap::serialPort = nullptr;

//CANTalon* RobotMap::shooterLeftFront;
//CANTalon* RobotMap::shooterRightBack;
//CANTalon* RobotMap::shooterRightFront;
//CANTalon* RobotMap::shooterLeftBack;

CANTalon* RobotMap::shooterMotor1;
CANTalon* RobotMap::shooterMotor2;

SpeedController* RobotMap::feeder;
AnalogInput* RobotMap::feederSensor;

CANTalon* RobotMap::indexMotor;
CANTalon* RobotMap::pickup;

CANTalon* RobotMap::armMotor;

CANTalon* RobotMap::winchMotor;
AnalogInput* RobotMap::winchPot;
PIDController* RobotMap::shooterWinch;

AHRS* RobotMap::imu = nullptr;

#define CONTINUOUS true
#define P 1.1 // 0.7
#define I 0.0
#define D 0.1 // 0.0
#define F 0.0
#define driveP 0.3
#define driveI 0.0
#define driveD 0.0
//#define driveF 0.1 //14 tooth gear
#define driveF 0.108 // 13 tooth gear
#define pdriveP 1.5
#define pdriveI 0.01
#define pdriveIZone 100.0
#define pdriveD 0.0
#define pdriveF 0.0
#define POTMIN 0.0
#define POTMAX 5.0
#define STEERPOW 1.0
#define TOLERANCE 0.1
#define PERIOD .02
#define RATIO 1

#define FLD 1
#define FLP 2
#define FLS 5

#define FRD 2
#define FRP 5
#define FRS 6

#define RLD 3
#define RLP 3
#define RLS 7

#define RRD 4
#define RRP 4
#define RRS 8

void RobotMap::Initialize() {
	// LiveWindow* lw = LiveWindow::GetInstance();

	// serialPort = new SerialPort(57600, SerialPort::kOnboard);
	imu = new AHRS(SPI::kMXP); // SPI::kOnboardCS0);//serialPort, 100);

	////////////////////////////////////
	////Front Left Wheel////////////////
	////////////////////////////////////
	driveTrainFrontLeftDrive = new CANTalon(FLD);
	driveTrainFrontLeftDrive->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	driveTrainFrontLeftDrive->ConfigPeakOutputVoltage(12.0, -12.0);
	driveTrainFrontLeftDrive->SetP(driveP);
	driveTrainFrontLeftDrive->SetI(driveI);
	driveTrainFrontLeftDrive->SetD(driveD);
	driveTrainFrontLeftDrive->SetF(driveF);
	driveTrainFrontLeftDrive->SetSensorDirection(true);
	driveTrainFrontLeftDrive->SelectProfileSlot(1);
	driveTrainFrontLeftDrive->SetP(pdriveP);
	driveTrainFrontLeftDrive->SetI(pdriveI);
	driveTrainFrontLeftDrive->SetIzone(pdriveIZone);
	driveTrainFrontLeftDrive->SetD(pdriveD);
	driveTrainFrontLeftDrive->SetF(pdriveF);
	driveTrainFrontLeftSteer = new CANTalon(FLS);
	driveTrainFrontLeftSteer->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	driveTrainFrontLeftPos = new AnalogChannelVolt(FLP, true, RATIO, driveTrainFrontLeftSteer);
	driveTrainFrontLeft = new PIDController(P, I, D, F, driveTrainFrontLeftPos, driveTrainFrontLeftSteer, PERIOD);
	driveTrainFrontLeft->SetContinuous(CONTINUOUS);
	driveTrainFrontLeft->SetAbsoluteTolerance(TOLERANCE);
	driveTrainFrontLeft->SetOutputRange(-STEERPOW, STEERPOW);

	////////////////////////////////////
	////Front Right Wheel///////////////
	////////////////////////////////////
	driveTrainFrontRightDrive = new CANTalon(FRD);
	driveTrainFrontRightDrive->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	driveTrainFrontRightDrive->ConfigPeakOutputVoltage(12.0, -12.0);
	driveTrainFrontRightDrive->SetP(driveP);
	driveTrainFrontRightDrive->SetI(driveI);
	driveTrainFrontRightDrive->SetD(driveD);
	driveTrainFrontRightDrive->SetF(driveF);
	driveTrainFrontRightDrive->SetSensorDirection(true);
	driveTrainFrontRightDrive->SelectProfileSlot(1);
	driveTrainFrontRightDrive->SetP(pdriveP);
	driveTrainFrontRightDrive->SetI(pdriveI);
	driveTrainFrontRightDrive->SetIzone(pdriveIZone);
	driveTrainFrontRightDrive->SetD(pdriveD);
	driveTrainFrontRightDrive->SetF(pdriveF);
	driveTrainFrontRightSteer = new CANTalon(FRS);
	driveTrainFrontRightSteer->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	driveTrainFrontRightPos = new AnalogChannelVolt(FRP, true, RATIO, driveTrainFrontRightSteer);
	driveTrainFrontRight = new PIDController(P, I, D, F, driveTrainFrontRightPos, driveTrainFrontRightSteer, PERIOD);
	driveTrainFrontRight->SetContinuous(CONTINUOUS);
	driveTrainFrontRight->SetAbsoluteTolerance(TOLERANCE);
	driveTrainFrontRight->SetOutputRange(-STEERPOW, STEERPOW);

	////////////////////////////////////
	////Rear Left Wheel/////////////////
	////////////////////////////////////
	driveTrainRearLeftDrive = new CANTalon(RLD);
	driveTrainRearLeftDrive->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	driveTrainRearLeftDrive->ConfigPeakOutputVoltage(12.0, -12.0);
	driveTrainRearLeftDrive->SetP(driveP);
	driveTrainRearLeftDrive->SetI(driveI);
	driveTrainRearLeftDrive->SetD(driveD);
	driveTrainRearLeftDrive->SetF(driveF);
	driveTrainRearLeftDrive->SetSensorDirection(true);
	driveTrainRearLeftDrive->SelectProfileSlot(1);
	driveTrainRearLeftDrive->SetP(pdriveP);
	driveTrainRearLeftDrive->SetI(pdriveI);
	driveTrainRearLeftDrive->SetIzone(pdriveIZone);
	driveTrainRearLeftDrive->SetD(pdriveD);
	driveTrainRearLeftDrive->SetF(pdriveF);
	driveTrainRearLeftSteer = new CANTalon(RLS);
	driveTrainRearLeftSteer->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	driveTrainRearLeftPos = new AnalogChannelVolt(RLP, true, RATIO, driveTrainRearLeftSteer);
	driveTrainRearLeft = new PIDController(P, I, D, F, driveTrainRearLeftPos, driveTrainRearLeftSteer, PERIOD);
	driveTrainRearLeft->SetContinuous(CONTINUOUS);
	driveTrainRearLeft->SetAbsoluteTolerance(TOLERANCE);
	driveTrainRearLeft->SetOutputRange(-STEERPOW, STEERPOW);

	////////////////////////////////////
	////Rear Right Wheel////////////////
	////////////////////////////////////
	driveTrainRearRightDrive = new CANTalon(RRD);
	driveTrainRearRightDrive->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	driveTrainRearRightDrive->ConfigPeakOutputVoltage(12.0, -12.0);
	driveTrainRearRightDrive->SetP(driveP);
	driveTrainRearRightDrive->SetI(driveI);
	driveTrainRearRightDrive->SetD(driveD);
	driveTrainRearRightDrive->SetF(driveF);
	driveTrainRearRightDrive->SetSensorDirection(true);
	driveTrainRearRightDrive->SelectProfileSlot(1);
	driveTrainRearRightDrive->SetP(pdriveP);
	driveTrainRearRightDrive->SetI(pdriveI);
	driveTrainRearRightDrive->SetIzone(pdriveIZone);
	driveTrainRearRightDrive->SetD(pdriveD);
	driveTrainRearRightDrive->SetF(pdriveF);
	driveTrainRearRightSteer = new CANTalon(RRS);
	driveTrainRearRightSteer->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	driveTrainRearRightPos = new AnalogChannelVolt(RRP, true, RATIO, driveTrainRearRightSteer);
	driveTrainRearRight = new PIDController(P, I, D, F, driveTrainRearRightPos, driveTrainRearRightSteer, PERIOD);
	driveTrainRearRight->SetContinuous(CONTINUOUS);
	driveTrainRearRight->SetAbsoluteTolerance(TOLERANCE);
	driveTrainRearRight->SetInputRange(POTMIN, POTMAX);
	driveTrainRearRight->SetOutputRange(-STEERPOW, STEERPOW);

	i2c = new I2C((I2C::Port)1, 0x04);

    //shooterLeftFront = new CANTalon(11);
    //shooterLeftBack = new CANTalon(30);
    //shooterRightFront = new CANTalon(12);
    //shooterRightBack = new CANTalon(29);

    shooterMotor1 = new CANTalon(11);
    shooterMotor2 = new CANTalon(12);

	feeder = new Talon(0);
	feederSensor = new AnalogInput(0);
	feederSensor->SetAverageBits(2);
	feederSensor->SetOversampleBits(4);
	feederSensor->SetSampleRate(1000);

	armMotor = new CANTalon(6);

	pickup = new CANTalon(10);
	indexMotor = new CANTalon(9);
	winchMotor = new CANTalon(5);
	winchPot = new AnalogInput(1);
	shooterWinch = new PIDController(1.5, 0.1, 0.1, 0.0, winchPot, winchMotor);
	shooterWinch->SetOutputRange(-0.5, 0.5);
	shooterWinch->SetAbsoluteTolerance(0.015);
}
