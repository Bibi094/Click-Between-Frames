/*
Reads input straight from evdev (/dev/input/event*), the same way the Linux helper does,
so inputs get their real hardware timestamps instead of the ones GD assigns at the start of the frame.

libevdev isn't available on Android, so this talks to the evdev ioctls directly.
Unlike the Linux helper there is no separate process / shared memory: a thread in GD reads the devices
and the main thread picks the events up in androidCheckInputs().

Note: /dev/input is normally not readable by regular apps (needs root or an SELinux-permissive setup).
If no touchscreen can be opened, linuxNative stays false and CBF keeps using GD's own input queue.
*/
#include "includes.hpp"
#include "android.hpp"

// must come after the Geode headers, the KEY_* macros in here clash with cocos' enumKeyCodes
#include <linux/input.h>

#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <sys/inotify.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* INPUT_DIR = "/dev/input/";
constexpr size_t MAX_PENDING = 256; // events kept while not in a level, oldest are dropped
constexpr int MAX_SLOTS = 32;

enum class Kind { Touchscreen, Mouse };

struct Device {
	int fd = -1;
	std::string path;
	Kind kind = Kind::Touchscreen;
	bool clockOk = false;      // kernel timestamps this device with CLOCK_MONOTONIC
	bool hasBtnTouch = false;  // otherwise contacts are counted through ABS_MT_TRACKING_ID
	bool pressed = false;
	bool resync = false;       // got SYN_DROPPED, ignore events until the next SYN_REPORT
	int slot = 0;
	int contacts = 0;
	std::array<bool, MAX_SLOTS> slotActive{};
};

std::mutex pendingMutex;
std::vector<PlayerButtonCommand> pending;

bool testBit(int bit, const unsigned char* bits) {
	return bits[bit / 8] & (1 << (bit % 8));
}

double monotonicNow() {
	timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

// seconds on the same clock as getCurrentTimestamp() (CLOCK_MONOTONIC)
double eventTime(const Device& d, const input_event& e) {
	double t = e.time.tv_sec + e.time.tv_usec / 1e6;
	if (!d.clockOk) {
		// device is stuck on CLOCK_REALTIME, shift it over to CLOCK_MONOTONIC
		timespec rt;
		clock_gettime(CLOCK_REALTIME, &rt);
		t -= (rt.tv_sec + rt.tv_nsec / 1e9) - monotonicNow();
	}
	return t;
}

void emit(double time, bool press) {
	PlayerButtonCommand cmd;
	cmd.m_button = PlayerButton::Jump;
	cmd.m_isPush = press;
	cmd.m_isPlayer2 = false;
	cmd.m_timestamp = time;

	std::lock_guard lock(pendingMutex);
	if (pending.size() >= MAX_PENDING) pending.erase(pending.begin());
	pending.push_back(cmd);
}

void setPressed(Device& d, bool press, double time) {
	if (d.pressed == press) return;
	d.pressed = press;
	emit(time, press);
}

// after SYN_DROPPED we may have missed a release, so ask the kernel what the real state is
void resyncState(Device& d) {
	if (d.kind == Kind::Mouse || d.hasBtnTouch) {
		unsigned char keys[KEY_MAX / 8 + 1] = {};
		if (ioctl(d.fd, EVIOCGKEY(sizeof(keys)), keys) >= 0) {
			int code = d.kind == Kind::Mouse ? BTN_LEFT : BTN_TOUCH;
			setPressed(d, testBit(code, keys), monotonicNow());
		}
	}
	else {
		d.slotActive.fill(false);
		d.contacts = 0;
		setPressed(d, false, monotonicNow());
	}
}

void handleEvent(Device& d, const input_event& e) {
	if (e.type == EV_SYN) {
		if (e.code == SYN_DROPPED) d.resync = true;
		else if (e.code == SYN_REPORT && d.resync) {
			d.resync = false;
			resyncState(d);
		}
		return;
	}
	if (d.resync) return;

	if (d.kind == Kind::Mouse) {
		if (e.type == EV_KEY && e.code == BTN_LEFT && e.value != 2) setPressed(d, e.value != 0, eventTime(d, e));
		return;
	}

	// touchscreen
	if (d.hasBtnTouch) {
		if (e.type == EV_KEY && e.code == BTN_TOUCH) setPressed(d, e.value != 0, eventTime(d, e));
		return;
	}

	if (e.type != EV_ABS) return;
	if (e.code == ABS_MT_SLOT) {
		d.slot = std::clamp(e.value, 0, MAX_SLOTS - 1);
	}
	else if (e.code == ABS_MT_TRACKING_ID) {
		bool down = e.value >= 0;
		bool active = d.slotActive[d.slot];
		if (down && !active) {
			d.slotActive[d.slot] = true;
			d.contacts++;
		}
		else if (!down && active) {
			d.slotActive[d.slot] = false;
			d.contacts--;
		}
		setPressed(d, d.contacts > 0, eventTime(d, e));
	}
}

/*
decide what a device is. returns false if we have no use for it.
touchscreens are accepted on any bus (on phones they sit on I2C/SPI/host buses),
the bus filter only applies to mice, to skip internal devices like gpio-keys.
*/
bool classify(int fd, Kind& kind, bool& hasBtnTouch) {
	unsigned char evBits[EV_MAX / 8 + 1] = {};
	unsigned char keyBits[KEY_MAX / 8 + 1] = {};
	unsigned char absBits[ABS_MAX / 8 + 1] = {};
	unsigned char propBits[INPUT_PROP_MAX / 8 + 1] = {};

	if (ioctl(fd, EVIOCGBIT(0, sizeof(evBits)), evBits) < 0) return false;
	if (testBit(EV_KEY, evBits)) ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keyBits)), keyBits);
	if (testBit(EV_ABS, evBits)) ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(absBits)), absBits);
	ioctl(fd, EVIOCGPROP(sizeof(propBits)), propBits); // may fail on old kernels, then propBits stays empty

	bool direct = testBit(INPUT_PROP_DIRECT, propBits);
	bool multitouch = testBit(EV_ABS, evBits) && testBit(ABS_MT_POSITION_X, absBits);
	bool trackingId = testBit(EV_ABS, evBits) && testBit(ABS_MT_TRACKING_ID, absBits);
	bool btnTouch = testBit(EV_KEY, evBits) && testBit(BTN_TOUCH, keyBits);
	bool toolFinger = testBit(EV_KEY, evBits) && testBit(BTN_TOOL_FINGER, keyBits); // touchpads

	// old Android touch drivers often don't set INPUT_PROP_DIRECT, so fall back to a heuristic
	bool touchscreen = direct || (multitouch && !toolFinger);
	if (touchscreen) {
		if (!btnTouch && !trackingId) return false; // no way to tell press from release
		kind = Kind::Touchscreen;
		hasBtnTouch = btnTouch;
		return true;
	}

	bool mouse = testBit(EV_KEY, evBits) && testBit(BTN_LEFT, keyBits) && testBit(EV_REL, evBits);
	if (mouse) {
		input_id id = {};
		if (ioctl(fd, EVIOCGID, &id) < 0) return false;
		if (id.bustype != BUS_USB && id.bustype != BUS_BLUETOOTH && id.bustype != BUS_I8042 && id.bustype != BUS_VIRTUAL) return false;
		kind = Kind::Mouse;
		return true;
	}

	return false;
}

bool addDevice(const std::string& path, int epollFd, std::vector<Device>& devices) {
	for (auto& d : devices) if (d.path == path) return false;

	int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd == -1) {
		// ENOENT: device vanished again, EACCES: not allowed to read it (yet, right after creation)
		static bool warnedAccess = false;
		if (errno == EACCES && !warnedAccess) {
			warnedAccess = true;
			log::warn("[CBF] No permission to read {} (and probably other input devices)", path);
		}
		else if (errno != ENOENT && errno != EACCES) {
			log::warn("[CBF] Failed to open {}: {}", path, strerror(errno));
		}
		return false;
	}

	Device d;
	if (!classify(fd, d.kind, d.hasBtnTouch)) {
		close(fd);
		return false;
	}

	// get timestamps in the same clock as getCurrentTimestamp()
	int clock = CLOCK_MONOTONIC;
	d.clockOk = ioctl(fd, EVIOCSCLOCKID, &clock) == 0;
	d.fd = fd;
	d.path = path;

	epoll_event ev = {};
	ev.events = EPOLLIN;
	ev.data.fd = fd;
	if (epoll_ctl(epollFd, EPOLL_CTL_ADD, fd, &ev) == -1) {
		log::warn("[CBF] Failed to add {} to epoll: {}", path, strerror(errno));
		close(fd);
		return false;
	}

	log::info("[CBF] Added {} device: {}", d.kind == Kind::Touchscreen ? "touchscreen" : "mouse", path);
	devices.push_back(std::move(d));
	return true;
}

void removeDevice(size_t index, int epollFd, std::vector<Device>& devices) {
	log::info("[CBF] Removed device: {}", devices[index].path);
	epoll_ctl(epollFd, EPOLL_CTL_DEL, devices[index].fd, nullptr);
	close(devices[index].fd);
	devices.erase(devices.begin() + index);
}

void handleInotify(int inotifyFd, int epollFd, std::vector<Device>& devices) {
	alignas(inotify_event) char buf[4096];
	ssize_t len;
	while ((len = read(inotifyFd, buf, sizeof(buf))) > 0) {
		for (ssize_t i = 0; i < len;) {
			auto* event = reinterpret_cast<inotify_event*>(buf + i);
			i += sizeof(inotify_event) + event->len;
			if (!event->len || strncmp(event->name, "event", 5) != 0) continue;

			std::string path = std::string(INPUT_DIR) + event->name;
			if (event->mask & IN_DELETE) {
				for (size_t n = 0; n < devices.size(); n++) {
					if (devices[n].path == path) {
						removeDevice(n, epollFd, devices);
						break;
					}
				}
			}
			else {
				// the device can't be opened right at creation, the permissions only get set with a later IN_ATTRIB
				addDevice(path, epollFd, devices);
			}
		}
	}
}

void readerLoop(std::vector<Device> devices, int epollFd, int inotifyFd) {
	epoll_event events[16];

	while (true) {
		int nfds = epoll_wait(epollFd, events, 16, -1);
		if (nfds == -1) {
			if (errno == EINTR) continue;
			log::error("[CBF] epoll_wait failed: {}", strerror(errno));
			break;
		}

		for (int n = 0; n < nfds; n++) {
			int fd = events[n].data.fd;
			if (fd == inotifyFd) {
				handleInotify(inotifyFd, epollFd, devices);
				continue;
			}

			auto it = std::find_if(devices.begin(), devices.end(), [fd](const Device& d) { return d.fd == fd; });
			if (it == devices.end()) continue; // removed earlier in this batch
			size_t index = it - devices.begin();

			bool remove = false;
			input_event buf[64];
			while (true) {
				ssize_t len = read(fd, buf, sizeof(buf));
				if (len < 0) {
					if (errno != EAGAIN && errno != EINTR) remove = true; // ENODEV when unplugged
					break;
				}
				if (len == 0) {
					remove = true;
					break;
				}
				for (size_t i = 0; i < len / sizeof(input_event); i++) handleEvent(devices[index], buf[i]);
			}
			if (remove) removeDevice(index, epollFd, devices);
		}
	}
}

} // namespace

void androidCheckInputs() {
	std::lock_guard lock(pendingMutex);
	if (pending.empty()) return;
	inputVector.insert(inputVector.end(), pending.begin(), pending.end());
	pending.clear();
}

void androidSetup() {
	int epollFd = epoll_create1(EPOLL_CLOEXEC);
	if (epollFd == -1) {
		log::error("[CBF] Failed to create epoll instance: {}", strerror(errno));
		return;
	}

	// hotplug is optional, if inotify isn't allowed we just keep the devices we found at startup
	int inotifyFd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
	if (inotifyFd != -1) {
		epoll_event ev = {};
		ev.events = EPOLLIN;
		ev.data.fd = inotifyFd;
		if (inotify_add_watch(inotifyFd, INPUT_DIR, IN_CREATE | IN_ATTRIB | IN_DELETE) < 0
			|| epoll_ctl(epollFd, EPOLL_CTL_ADD, inotifyFd, &ev) == -1) {
			close(inotifyFd);
			inotifyFd = -1;
		}
	}

	std::vector<Device> devices;
	if (DIR* dir = opendir(INPUT_DIR)) {
		while (dirent* entry = readdir(dir)) {
			if (strncmp(entry->d_name, "event", 5) == 0) addDevice(std::string(INPUT_DIR) + entry->d_name, epollFd, devices);
		}
		closedir(dir);
	}
	else {
		log::warn("[CBF] Failed to open {}: {}", INPUT_DIR, strerror(errno));
	}

	// without the touchscreen we'd drop GD's own touches (they're ignored once linuxNative is set), so stay vanilla
	bool hasTouchscreen = std::any_of(devices.begin(), devices.end(), [](const Device& d) { return d.kind == Kind::Touchscreen; });
	if (!hasTouchscreen) {
		log::warn("[CBF] No readable touchscreen in {}, using GD's input queue instead", INPUT_DIR);
		for (auto& d : devices) close(d.fd);
		close(epollFd);
		if (inotifyFd != -1) close(inotifyFd);
		return;
	}

	linuxNative = true;
	log::info("[CBF] Reading input directly from evdev");
	std::thread(readerLoop, std::move(devices), epollFd, inotifyFd).detach();
}
