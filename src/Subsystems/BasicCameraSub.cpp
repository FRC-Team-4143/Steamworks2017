#include "Subsystems/BasicCameraSub.h"
#include <CameraServer.h>

// ==========================================================================

BasicCameraSub::BasicCameraSub(const std::string &cameraName)
: frc::Subsystem("BasicCamera:" + cameraName), m_enabled(false) {
}

// ==========================================================================

BasicCameraSub::~BasicCameraSub() {
}

// ==========================================================================

void BasicCameraSub::Enable() {
	if (!IsEnabled()) {
		_InitializeCamera();
		m_enabled = true;
	}
}

// ==========================================================================

bool BasicCameraSub::IsEnabled() const {
	return m_enabled;
}

// ==========================================================================

void BasicCameraSub::_InitializeCamera() {
	CameraServer::GetInstance()->StartAutomaticCapture();
}

// ==========================================================================
