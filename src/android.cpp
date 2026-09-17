#include "includes.hpp"
#include "android.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

constexpr size_t RING_BUFFER_SIZE = 256;

struct __attribute__((packed)) SharedMemory {
	volatile uint32_t head;
	volatile uint32_t tail;
	volatile uint32_t error_flag;
	volatile uint32_t heartbeat;
	LinuxInputEvent events[RING_BUFFER_SIZE];
};

// Must match `MEMFD_NAME` in the `linux-input` root helper's `android.rs`
// exactly: that's how it finds this region, by scanning this process'
// /proc/<pid>/fd for a memfd whose name matches this.
constexpr const char* MEMFD_NAME = "cbf_shm";

// XInput SDK constants (from <xinput.h>, Windows-only); hardcoded here since
// they're just plain thresholds, to keep this logic identical to
// windows.cpp's CONTROLLER case.
constexpr int LEFT_THUMB_DEADZONE = 7849;
constexpr int RIGHT_THUMB_DEADZONE = 8689;
constexpr int TRIGGER_THRESHOLD = 30;

extern bool androidNative;

// Kept open (deliberately never closed) for the lifetime of the process:
// `linux-input --android` finds this region by locating this fd via
// /proc/<pid>/fd, so it must stay open and mapped for as long as we want
// input to keep working.
int shmFd = -1;
SharedMemory* pSharedMem = nullptr;

void androidHeartbeat() {
	if (pSharedMem) pSharedMem->heartbeat++;
}

void androidCheckInputs() {
	if (!pSharedMem) return;

	static std::unordered_map<int, enumKeyCodes> linuxToCCKey = {
		{ BTN_A, CONTROLLER_A },
		{ BTN_B, CONTROLLER_B },
		{ BTN_X, CONTROLLER_X },
		{ BTN_Y, CONTROLLER_Y },
		{ BTN_TL, CONTROLLER_LB },
		{ BTN_TR, CONTROLLER_RB },
		{ BTN_SELECT, CONTROLLER_Back },
		{ BTN_START, CONTROLLER_Start },
	};

	uint32_t h = pSharedMem->head;
	std::atomic_thread_fence(std::memory_order_acquire);
	uint32_t t = pSharedMem->tail;

	while (t != h) {
		const LinuxInputEvent& ev = pSharedMem->events[t & (RING_BUFFER_SIZE - 1)];
		t++;

		PlayerButtonCommand input;
		bool player1 = true;
		uint16_t scanCode = ev.code;
		int value = ev.value;

		switch (ev.deviceType) {
		case MOUSE:
		case TOUCHPAD:
			if (scanCode == BUTTON_LEFT) {
				input.m_button = PlayerButton::Jump;
			}
			else if (scanCode == BUTTON_RIGHT) {
				if (!enableRightClick) continue;
				input.m_button = PlayerButton::Jump;
				player1 = false;
			}
			else continue;
			break;
		case TOUCHSCREEN:
			if (scanCode == BTN_TOUCH) {
				input.m_button = PlayerButton::Jump;
			}
			else continue;
			break;
		case KEYBOARD:
			// TODO: external/USB-OTG keyboard support isn't implemented on
			// Android yet. windows.cpp maps a Linux scancode to a Windows
			// virtual-key code (via MapVirtualKeyExA) before comparing it
			// against inputBinds; there's no equivalent API here, and no
			// Linux-scancode -> enumKeyCodes table wired up in this file.
			// Touchscreen and controller input both work as normal.
			continue;
		case CONTROLLER: {
			int keyCode = -1;
			if (ev.type == EV_KEY) {
				keyCode = linuxToCCKey[scanCode];
			} else if (ev.type == EV_ABS) {
				bool continueLoop = false;
				auto analyze4Directions = [&] (int deadzone, enumKeyCodes negative, enumKeyCodes positive) {
					if (ev.value < -deadzone) {
						keyCode = negative;
						if (heldInputs.contains(negative)) {
							continueLoop = true;
						}
						value = Press;
					} else if (ev.value > deadzone) {
						keyCode = positive;
						if (heldInputs.contains(positive)) {
							continueLoop = true;
						}
						value = Press;
					} else {
						value = Release;
						if (heldInputs.contains(negative)) {
							keyCode = negative;
						} else if (heldInputs.contains(positive)) {
							keyCode = positive;
						} else {
							continueLoop = true;
						}
					}
				};

				switch (ev.code) {
				case ABS_X:
					analyze4Directions(LEFT_THUMB_DEADZONE, CONTROLLER_LTHUMBSTICK_LEFT, CONTROLLER_LTHUMBSTICK_RIGHT);
					break;
				case ABS_Y:
					analyze4Directions(LEFT_THUMB_DEADZONE, CONTROLLER_LTHUMBSTICK_UP, CONTROLLER_LTHUMBSTICK_DOWN);
					break;
				case ABS_RX:
					analyze4Directions(RIGHT_THUMB_DEADZONE, CONTROLLER_RTHUMBSTICK_LEFT, CONTROLLER_RTHUMBSTICK_RIGHT);
					break;
				case ABS_RY:
					analyze4Directions(RIGHT_THUMB_DEADZONE, CONTROLLER_RTHUMBSTICK_UP, CONTROLLER_RTHUMBSTICK_DOWN);
					break;
				case ABS_HAT0X:
					analyze4Directions(10, CONTROLLER_Left, CONTROLLER_Right);
					break;
				case ABS_HAT0Y:
					analyze4Directions(10, CONTROLLER_Up, CONTROLLER_Down);
					break;
				case ABS_Z:
					keyCode = CONTROLLER_LT;
					if (ev.value > TRIGGER_THRESHOLD) {
						value = Press;
					} else {
						value = Release;
					}
					break;
				case ABS_RZ:
					keyCode = CONTROLLER_RT;
					if (ev.value > TRIGGER_THRESHOLD) {
						value = Press;
					} else {
						value = Release;
					}
					break;
				}
				if (continueLoop) continue;
			}
			if (inputBinds[p1Jump].contains(keyCode)) input.m_button = PlayerButton::Jump;
			else if (inputBinds[p1Left].contains(keyCode)) input.m_button = PlayerButton::Left;
			else if (inputBinds[p1Right].contains(keyCode)) input.m_button = PlayerButton::Right;
			else {
				player1 = false;
				if (inputBinds[p2Jump].contains(keyCode)) input.m_button = PlayerButton::Jump;
				else if (inputBinds[p2Left].contains(keyCode)) input.m_button = PlayerButton::Left;
				else if (inputBinds[p2Right].contains(keyCode)) input.m_button = PlayerButton::Right;
				else continue;
			}
			if (value == Press) {
				if (heldInputs.contains(keyCode)) {
					continue;
				} else {
					heldInputs.emplace(keyCode);
				}
			} else {
				if (!heldInputs.contains(keyCode)) {
					continue;
				} else {
					heldInputs.erase(keyCode);
				}
			}
			break;
		}
		default:
			continue;
		}

		input.m_isPush = value;
		// `ev.time` is nanoseconds since CLOCK_MONOTONIC's epoch (the root
		// helper requests this clock specifically for Android via
		// EVIOCSCLOCKID), matching what getCurrentTimestamp() uses here.
		input.m_timestamp = (double)ev.time / 1'000'000'000.0;
		input.m_isPlayer2 = !player1;

		inputVector.emplace_back(input);
	}

	pSharedMem->tail = t;
}

void androidSetup() {
	// Not using the libc memfd_create() wrapper directly since it's only
	// available starting at a fairly recent Android API level; going
	// through the raw syscall works on any kernel that supports it.
	shmFd = memfd_create(MEMFD_NAME, 0);
	if (shmFd == -1) {
		log::error("Failed to create shared memory: {}", strerror(errno));
		return;
	}

	if (ftruncate(shmFd, sizeof(SharedMemory)) == -1) {
		log::error("Failed to size shared memory: {}", strerror(errno));
		close(shmFd);
		shmFd = -1;
		return;
	}

	pSharedMem = static_cast<SharedMemory*>(
		mmap(nullptr, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, shmFd, 0));
	if (pSharedMem == MAP_FAILED) {
		log::error("Failed to map shared memory: {}", strerror(errno));
		close(shmFd);
		shmFd = -1;
		pSharedMem = nullptr;
		return;
	}

	memset(pSharedMem, 0, sizeof(SharedMemory));

	// `linux-input --android` (run separately, as root -- this process has
	// no way to launch it itself) finds `shmFd` by searching for this
	// process by name and scanning its /proc/<pid>/fd for a memfd named
	// MEMFD_NAME, then reopening it. That means `shmFd` must be left open;
	// don't add a close() here like windows.cpp does with its file handle.

	androidNative = true;
	log::info("Android native input set up");
}
