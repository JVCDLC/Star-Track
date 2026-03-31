#ifndef REFACTORED_CONFIG_HPP
#define REFACTORED_CONFIG_HPP

#include <Arduino.h>

namespace refactored {

enum class MotorId : uint8_t {
  AXIS_DEC = 0,
  AXIS_RA = 1,
  AXIS_FOC = 2,
  COUNT = 3
};

enum class SwitchId : uint8_t {
  SW1 = 0,
  SW2 = 1,
  SW3 = 2,
  SW4 = 3,
  SW5 = 4,
  SW6 = 5,
  SW7 = 6,
  SW8 = 7,
  SW9 = 8,
  SW10 = 9,
  COUNT = 10
};

namespace pins {

// DEC encoder + BTS7960 pins
static const uint8_t DEC_ENC_A = 2;
static const uint8_t DEC_ENC_B = 3;
static const uint8_t DEC_RPWM = 6;
static const uint8_t DEC_LPWM = 7;
static const uint8_t DEC_REN = 22;
static const uint8_t DEC_LEN = 23;

// RA encoder + BTS7960 pins
static const uint8_t RA_ENC_A = 19;
static const uint8_t RA_ENC_B = 18;
static const uint8_t RA_RPWM = 9;
static const uint8_t RA_LPWM = 8;
static const uint8_t RA_REN = 24;
static const uint8_t RA_LEN = 25;

// Focus encoder + MD13S pins
static const uint8_t FOC_ENC_A = 50;  // Port B bit 2
static const uint8_t FOC_ENC_B = 51;  // Port B bit 3
static const uint8_t FOC_DIR = 10;
static const uint8_t FOC_PWM = 11;

// Limit switch pins (active-low)
static const uint8_t SW1 = A8;
static const uint8_t SW2 = A9;
static const uint8_t SW3 = A10;
static const uint8_t SW4 = A11;
static const uint8_t SW5 = A12;
static const uint8_t SW6 = A13;
static const uint8_t SW7 = A14;
static const uint8_t SW8 = A15;
static const uint8_t SW9 = 12;   // Port B bit 6
static const uint8_t SW10 = 13;  // Port B bit 7

}  // namespace pins

// Fast quadrature decoding table for 2-bit Gray transitions.
static const int8_t QUAD_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

}  // namespace refactored

#endif  // REFACTORED_CONFIG_HPP
