// Host-side Arduino stub for off-target simulation of AutoTunePID.
// Provides only the symbols the library consumes: millis() and constrain().
#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H

#include <stdint.h>
#include <math.h>

// Simulated millisecond clock, advanced by the harness before each update.
extern uint32_t g_simMillis;

inline uint32_t millis() { return g_simMillis; }

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#endif // ARDUINO_STUB_H
