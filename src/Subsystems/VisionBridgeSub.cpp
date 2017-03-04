#include "Subsystems/VisionBridgeSub.h"
#include "Modules/CastUtil.h"
#include "Modules/Logger.h"
#include "RobotMap.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <condition_variable>
#include <iostream>

const uint16_t DEFAULT_LISTENING_PORT = 4143; // TODO - Change to allowed port range

#define MAX_PACKET_SIZE 2048

// ==========================================================================

VisionBridgeSub::VisionBridgeSub()
:	VisionBridgeSub(DEFAULT_LISTENING_PORT) {
}

// ==========================================================================

VisionBridgeSub::VisionBridgeSub(uint16_t listeningPort)
:	frc::Subsystem("VisionBridgeSub"),
	_mutex(),
	_listeningPort(listeningPort),
	_gearPosition(0),
	_gearDistance(0),
	_boilerPosition(0),
	_boilerDistance(0),
	_zeroCounterGearPos(0),
	_zeroCounterGearDist(0),
	_zeroCounterBoilerPos(0),
	_zeroCounterBoilerDist(0),
	_debug(false),
	_listeningThread(&VisionBridgeSub::Listen, this),
	_autoAim(0) {
}

// ==========================================================================

void VisionBridgeSub::InitDefaultCommand() {
	//SetDefaultCommand(new MySpecialCommand());
}

// ==========================================================================

void VisionBridgeSub::EnableDebug(bool debug) {
	_debug = debug;
}

// ==========================================================================
// Side 0 is left
// Side 1 is right

double VisionBridgeSub::GetGearPosition() {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	if (_gearPosition != 0) {
		_autoAim = 0;
		return _gearPosition;
	}

	auto gyroYaw = RobotMap::imu->GetYaw();
	if (gyroYaw > 90) {
		_autoAim = -150;
	}
	else if (gyroYaw < -90) {
		_autoAim = 150;
	}
	return _autoAim;

}

// ==========================================================================

double VisionBridgeSub::GetGearDistance() {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	return _gearDistance;
}

// ==========================================================================

double VisionBridgeSub::GetBoilerPosition() {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	if (_boilerPosition != 0) {
		_autoAim = 0;
		return _boilerPosition;
	}

	auto gyroYaw = RobotMap::imu->GetYaw();
	if (gyroYaw > 90) {
		_autoAim = -150;
	}
	else if (gyroYaw < -90) {
		_autoAim = 150;
	}
	return _autoAim;

}

// ==========================================================================

double VisionBridgeSub::GetBoilerDistance() {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	return _boilerDistance;
}

// ==========================================================================

void VisionBridgeSub::DebugOutput(std::string packet) {
	LOG("VisionPacket=" + packet);
}

// ==========================================================================

void VisionBridgeSub::Listen() {
	sockaddr_in serverAddr; // server's socket address
	sockaddr_in clientAddr; // client's socket address
	socklen_t sockAddrSize; // size of socket address structure
	int sockListener;       // socket file descriptor
	char buf[MAX_PACKET_SIZE + 1];

	// Set up the local address.
	sockAddrSize = sizeof(sockaddr_in);
	bzero(&serverAddr, sockAddrSize);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(_listeningPort);
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	std::cout << GetName() << " Starting UDP listener on port " << _listeningPort << std::endl;

	// Create a UDP socket.
	sockListener = socket(AF_INET, SOCK_DGRAM, 0);
	//if ((sockListener = socket(AF_INET, SOCK_DGRAM, 0)) == ERROR) {
	//    perror("socket");
	//    return;
	//}
	std::cout << GetName() << " Created socket" << std::endl;

	// Bind socket to local address.
	bind(sockListener, S_CAST(sockaddr *, S_CAST(void *, &serverAddr)), sockAddrSize);
	//if (bind(sockListener, S_CAST(sockaddr*, S_CAST(void*, &serverAddr)), sockAddrSize) == ERROR) {
	//    perror("bind");
	//    close(sockListener);
	//    return;
	//}
	std::cout << GetName() << " Bound socket" << std::endl;

	for (;;) {
		//if (_debug) std::cout << "[DEBUG] " << GetName() << " Waiting on port" << std::endl;
		int recvlen = recvfrom(sockListener, buf, sizeof(buf) - 1, 0,
				S_CAST(sockaddr *, S_CAST(void *, &clientAddr)),
				&sockAddrSize);
		//if (_debug) std::cout << "[DEBUG] " << GetName() << " Received " << recvlen << " bytes" << std::endl;
		if (recvlen > 0) {
			buf[recvlen] = 0;
			ParsePacket(buf);
		}
	}

	close(sockListener);
}

// ==========================================================================

void VisionBridgeSub::ParsePacket(char packet[]) {
	if (_debug) {
		DebugOutput(packet);
	}
	try {
		char* pch = std::strtok(packet, " ");
		auto boilerPos = std::stod(pch);
		SetBoilerPosition(boilerPos);
		pch = std::strtok(nullptr, " ");
		auto boilerDist = std::stod(pch);
		SetBoilerDistance(boilerDist);
		pch = std::strtok(nullptr, " ");
		auto gearPosition = std::stod(pch);
		SetGearPosition(gearPosition);
		pch = std::strtok(nullptr, " ");
		auto gearDist = std::stod(pch);
		SetGearDistance(gearDist);
	}
	catch (...) {
	}
}

// ==========================================================================

void VisionBridgeSub::SetGearPosition(double position) {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	if (position != 0.0) {
		_gearPosition = position;
		_zeroCounterGearPos = 0;
	}
	else {
		_zeroCounterGearPos++;
		if (_zeroCounterGearPos > 10) {
			_gearPosition = position;
		}
	}
}

// ==========================================================================

void VisionBridgeSub::SetGearDistance(double distance) {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	if (distance != 0.0) {
		_gearDistance = distance;
		_zeroCounterGearDist = 0;
	}
	else {
		_zeroCounterGearDist++;
		if (_zeroCounterGearDist > 10) {
			_gearDistance = distance;
		}
	}
}

// ==========================================================================
void VisionBridgeSub::SetBoilerPosition(double position) {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	if (position != 0.0) {
		_boilerPosition = position;
		_zeroCounterBoilerPos = 0;
	}
	else {
		_zeroCounterBoilerPos++;
		if (_zeroCounterBoilerPos > 10) {
			_boilerPosition = position;
		}
	}
}

// ==========================================================================


void VisionBridgeSub::SetBoilerDistance(double distance) {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	if (distance != 0.0) {
		_boilerDistance = distance;
		_zeroCounterBoilerDist = 0;
	}
	else {
		_zeroCounterBoilerDist++;
		if (_zeroCounterBoilerDist > 10) {
			_boilerDistance = distance;
		}
	}
}
