#include "bts7960_motor.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

namespace refactored {

/**
 * @brief Constructor for BTS7960Motor.
 * 
 * Stores motor configuration and delegates to base class constructor.
 * The all all configuration (PID, limits, calibration) is inherited from MotorBase.
 * Pin configuration is stored for later use in setupDriverPins() and applyDriverPwm().
 */
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

/**
 * @brief Set up BTS7960 driver pins as outputs and initialize to safe state.
 * 
 * Called by begin() to configure hardware:
 * 1. Set all pins as OUTPUT
 * 2. Set enable pins HIGH (enables the driver)
 * 3. Zero all PWM outputs (stops motor)
 * 
 * After this, the motor is ready to receive PWM commands.
 */
void BTS7960Motor::setupDriverPins() {
  // Configure enable pins
  pinMode(m_pins.enableForward, OUTPUT);
  pinMode(m_pins.enableReverse, OUTPUT);
  
  // Configure PWM pins
  pinMode(m_pins.pwmForward, OUTPUT);
  pinMode(m_pins.pwmReverse, OUTPUT);

  // Enable the driver (active-high logic)
  digitalWrite(m_pins.enableForward, HIGH);
  digitalWrite(m_pins.enableReverse, HIGH);
  
  // Initialize PWM to zero (motor stopped)
  analogWrite(m_pins.pwmForward, 0);
  analogWrite(m_pins.pwmReverse, 0);
}

/**
 * @brief Apply signed PWM to control motor direction and speed.
 * @param signedPwm Signed value from -255 to +255
 * 
 * Implements dual-PWM H-bridge control:
 * - Positive PWM: Forward direction at magnitude
 * - Negative PWM: Reverse direction at magnitude
 * - Zero: Motor stopped (coasts, no holding torque)
 * 
 * Example:
 * - signedPwm = +128: Forward PWM = 128, Reverse PWM = 0
 * - signedPwm = -128: Forward PWM = 0, Reverse PWM = 128
 * - signedPwm = 0: Both PWM = 0 (stopped)
 * 
 * This approach provides smooth transitions between directions because only
 * one PWM channel is active at any time, preventing shoot-through current.
 */
void BTS7960Motor::applyDriverPwm(int16_t signedPwm) {
  if (signedPwm > 0) {
    // Forward direction: positive PWM
    analogWrite(m_pins.pwmForward, static_cast<uint8_t>(signedPwm));
    analogWrite(m_pins.pwmReverse, 0);  // Ensure reverse is off
  } else if (signedPwm < 0) {
    // Reverse direction: negative PWM (negate to positive for analogWrite)
    analogWrite(m_pins.pwmForward, 0);  // Ensure forward is off
    analogWrite(m_pins.pwmReverse, static_cast<uint8_t>(-signedPwm));
  } else {
    // Stopped: both PWM channels off
    analogWrite(m_pins.pwmForward, 0);
    analogWrite(m_pins.pwmReverse, 0);
  }
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
