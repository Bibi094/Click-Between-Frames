#pragma once

#include <Geode/Geode.hpp>
#include "linuxeventcodes.hpp"

#include <cstdint>

// Mirrors windows.hpp's DeviceType/LinuxInputEvent, but with plain
// fixed-width types instead of Win32's LARGE_INTEGER/USHORT (Android has
// neither), matching the packed layout the `linux-input` root helper
// writes byte-for-byte.
enum DeviceType : int8_t {
    MOUSE,
    TOUCHPAD,
    KEYBOARD,
    TOUCHSCREEN,
    CONTROLLER,
    UNKNOWN
};

struct __attribute__((packed)) LinuxInputEvent {
    int64_t time;
    uint16_t type;
    uint16_t code;
    int32_t value;
    DeviceType deviceType;
};

// True once `androidSetup()` has successfully created and mapped the
// shared-memory region.
extern bool androidNative;

void androidSetup();
void androidCheckInputs();
void androidHeartbeat();
