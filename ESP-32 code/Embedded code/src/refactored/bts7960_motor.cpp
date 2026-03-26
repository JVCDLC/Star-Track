#include "bts7960_motor.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

namespace refactored {

BTS7960Motor::BTS7960Motor(const char* name,
                           MotorId encoderId,
                           const Bts7960Pins& pins,
                           const PidConfig& pid,
                           const MotorRuntimeConfig& runtime,
                           const LimitConfig& limits,
                           const CalibrationConfig& calibration,
                           const PrecisionConfig& precision)
    : MotorBase(name, encoderId, pid, runtime, limits, calibration, precision),
      m_pins(pins) {}

void BTS7960Motor::setupDriverPins() {
  pinMode(m_pins.enableForward, OUTPUT);
  pinMode(m_pins.enableReverse, OUTPUT);
  pinMode(m_pins.pwmForward, OUTPUT);
  pinMode(m_pins.pwmReverse, OUTPUT);

  digitalWrite(m_pins.enableForward, HIGH);
  digitalWrite(m_pins.enableReverse, HIGH);
  analogWrite(m_pins.pwmForward, 0);
  analogWrite(m_pins.pwmReverse, 0);
}

void BTS7960Motor::applyDriverPwm(int16_t signedPwm) {
  if (signedPwm > 0) {
    analogWrite(m_pins.pwmForward, static_cast<uint8_t>(signedPwm));
    analogWrite(m_pins.pwmReverse, 0);
  } else if (signedPwm < 0) {
    analogWrite(m_pins.pwmForward, 0);
    analogWrite(m_pins.pwmReverse, static_cast<uint8_t>(-signedPwm));
  } else {
    analogWrite(m_pins.pwmForward, 0);
    analogWrite(m_pins.pwmReverse, 0);
  }
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
