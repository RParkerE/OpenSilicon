#pragma once
#include <cstdint>

namespace Domain {

	/**
	 * Enum of all display pages.
	 * The DisplayService selects the active page based on MachineState.
	 */
	enum class DisplayPage : uint8_t {
		Status,
		Recipe,
		ActiveProcess,
		Fault
	};
}
