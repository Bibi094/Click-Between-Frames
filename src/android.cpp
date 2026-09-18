#include "includes.hpp"
#include "android.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
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

// androidNative is defined in main.cpp (alongside linuxNative) and declared
// extern in android.hpp; set true once setup below succeeds.

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

	// Last known primary-contact position, normalized to 0..65535 per axis
	// by linux-input (see devices.rs); -1 means not known yet. Updated by
	// ABS_X/ABS_Y reports; read once a touch's press is resolved -- see
	// `resolvePendingTouchPress` below -- to hit-test against UI buttons
	// (isTouchOnUi() in main.cpp).
	static int touchX = -1, touchY = -1;
	// A BTN_TOUCH press seen but not yet resolved into either a forwarded
	// gameplay press or a dropped UI tap -- see the comment where this is
	// set, below. `pendingTouchWasUi` holds the outcome once resolved, so
	// the eventual matching release (which can arrive in a *later* call to
	// this function, for a held touch) is treated consistently with its
	// press.
	static bool pendingTouchPress = false;
	static bool pendingTouchWasUi = false;
	static double pendingTouchPressTime = 0.0;

	// Resolves a still-pending press using whatever position is known by
	// now, forwarding it as a gameplay press unless it lands on a UI
	// button. Safe to call even when nothing is pending (no-op).
	auto resolvePendingTouchPress = [&]() {
		if (!pendingTouchPress) return;
		pendingTouchPress = false;
		pendingTouchWasUi = isTouchOnUi(touchX, touchY);
		if (pendingTouchWasUi) return;
		PlayerButtonCommand press{};
		press.m_button = PlayerButton::Jump;
		press.m_isPush = true;
		press.m_isPlayer2 = false;
		press.m_timestamp = pendingTouchPressTime;
		inputVector.emplace_back(press);
	};

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
				if (value) {
					// The kernel's single-touch-compatibility emulation
					// (input_mt_report_pointer_emulation() in the
					// kernel's input-mt.c) reports BTN_TOUCH *before* the
					// corresponding ABS_X/ABS_Y for a new contact, so this
					// touch's actual position isn't necessarily reflected
					// in touchX/touchY yet. Defer the UI hit-test (and
					// this press's very existence, as far as gameplay is
					// concerned) until it resolves: either once a
					// position update arrives and this same touch is
					// released, or once this whole batch has been
					// drained, below, whichever comes first.
					pendingTouchPress = true;
					pendingTouchPressTime = (double)ev.time / 1'000'000'000.0;
					continue;
				}
				// Release: resolve any still-pending press first, using
				// whatever position is known by now (its ABS_X/ABS_Y
				// update, if any, is guaranteed to have already been
				// processed, since it can only have arrived before this
				// release). Only forward the release if the press wasn't
				// suppressed as a UI tap -- otherwise the game never saw
				// a press for this to release.
				resolvePendingTouchPress();
				if (pendingTouchWasUi) continue;
				input.m_button = PlayerButton::Jump;
			}
			else if (scanCode == ABS_X) { touchX = value; continue; }
			else if (scanCode == ABS_Y) { touchY = value; continue; }
			else continue;
			break;
		case KEYBOARD: {
			// `scanCode` has already been resolved to the equivalent
			// enumKeyCodes value by `linux-input` (via the `hairetsu`
			// crate; see its `devices.rs` for the full mapping) -- there's
			// no Windows API to do that step here, unlike windows.cpp's
			// MapVirtualKeyExA call, so all of it happens on the Rust
			// side instead.
			int keyCode = scanCode;
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
			break;
		}
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

	// The batch may have ended with a press still pending (e.g. the touch
	// is being held and its release hasn't happened yet): resolve it now
	// rather than leaving the decision -- and the jump it represents --
	// hanging until some future call. By this point any ABS_X/ABS_Y update
	// for it has already been processed above, so the position is as
	// current as it's going to get.
	resolvePendingTouchPress();

	pSharedMem->tail = t;
}

void androidSetup() {
	// MFD_ALLOW_SEALING is required for the F_ADD_SEALS call below to work
	// at all: without it, the kernel implicitly applies F_SEAL_SEAL at
	// creation time, which then makes *any* later sealing attempt fail
	// with EPERM.
	shmFd = memfd_create(MEMFD_NAME, MFD_ALLOW_SEALING);
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

	// Seal the region's size: since it's exposed (deliberately, for
	// `linux-input --android` to find and reopen) via /proc/<pid>/fd to any
	// root process, this stops anything from ever shrinking it out from
	// under either side's mmap (which would raise SIGBUS) or growing it,
	// and F_SEAL_SEAL locks that seal set in permanently. Deliberately
	// *not* sealing writes (F_SEAL_WRITE/F_SEAL_FUTURE_WRITE): F_SEAL_WRITE
	// would block `linux-input`'s own later mmap(PROT_WRITE) call (it opens
	// and maps this region well after this point, whenever the user runs
	// it), which would break the entire mechanism; F_SEAL_FUTURE_WRITE only
	// blocks the write()/pwrite() syscalls, which nothing here ever calls
	// on this fd anyway, so it wouldn't add any real protection. Sealing is
	// a hardening measure, not a functional requirement, so a failure here
	// is logged but not fatal.
	if (fcntl(shmFd, F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_SEAL) == -1) {
		log::error("Failed to seal shared memory: {}", strerror(errno));
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
