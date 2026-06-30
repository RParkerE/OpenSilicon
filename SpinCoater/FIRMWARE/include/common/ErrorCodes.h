#pragma once
#include <cstdint>

namespace Common {
	
	enum class ErrorCode : uint16_t {
		None = 0,
		SpinMotorFault,
		PumpMotorFault,
		ZAxisMotorFault,
		OverTemperature,
		EmergencyStopPressed,
		SensorFault,
		CommunicationFault
	};
}
