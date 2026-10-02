#pragma once

// true when inputs are read straight from evdev instead of coming from GD's own input queue
// (shared with the Windows/Wine-on-Linux path, defined in main.cpp)
extern bool linuxNative;

// scans /dev/input and starts the reader thread; leaves linuxNative false if no usable device could be opened
void androidSetup();

// moves the events collected by the reader thread into inputVector
void androidCheckInputs();
