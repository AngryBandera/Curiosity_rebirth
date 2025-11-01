// This component previously contained an implementation of WheelDriver which
// duplicated the implementation in main/motors/motors.cpp. To avoid
// multiple-definition linker errors, the implementation is kept in
// main/motors/motors.cpp; this file intentionally contains no definitions.
// Keep the source present so the component remains discoverable by CMake.
#include "motors.h"
