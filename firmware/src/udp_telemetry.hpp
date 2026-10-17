#ifndef UDP_TELEMETRY_HPP
#define UDP_TELEMETRY_HPP

#include <Arduino.h>

extern const char* Cutoff_reason;

void udp_telemetry_init(void);
void set_cutoff_reason(const char* reason);

#endif
