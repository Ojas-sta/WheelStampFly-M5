#include "udp_telemetry.hpp"
#include <Arduino.h>

const char* Cutoff_reason = "NORMAL (BOOT)";

void set_cutoff_reason(const char* reason) {
    Cutoff_reason = reason;
}

void udp_telemetry_init(void) {
    // UDP telemetry temporarily disabled per user request
}
