// dmxmap.h
#ifndef DMXMAP_H
#define DMXMAP_H

#include <stdint.h> // For uint16_t

// DMX Channel Offsets (relative to a given base DMX address)
inline uint16_t DMX_SERVO_1(uint16_t base) { return base + 0; }
inline uint16_t DMX_SERVO_2(uint16_t base) { return base + 1; }
inline uint16_t DMX_SERVO_3(uint16_t base) { return base + 2; }
inline uint16_t DMX_SERVO_4(uint16_t base) { return base + 3; }

inline uint16_t DMX_MOTOR_A(uint16_t base) { return base + 4; }
inline uint16_t DMX_MOTOR_B(uint16_t base) { return base + 5; }

inline uint16_t DMX_STEPPER_SPEED(uint16_t base) { return base + 6; }
inline uint16_t DMX_STEPPER_POSITION(uint16_t base) { return base + 7; }

inline uint16_t DMX_RELAY_1(uint16_t base) { return base + 8; }
inline uint16_t DMX_RELAY_2(uint16_t base) { return base + 9; }

inline uint16_t DMX_LAST(uint16_t base) { return base + 9; }

#endif // DMXMAP_H
