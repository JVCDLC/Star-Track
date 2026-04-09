#ifndef BTS7960_MOTOR_HPP
#define BTS7960_MOTOR_HPP

#include "motor_base.hpp"

namespace refactored {

/**
 * @brief Pin configuration for BTS7960 dual-H-bridge motor driver.
 * 
 * The BTS7960 is a dual-channel H-bridge used for RA and DEC motors.
 * It provides independent PWM and enable control for forward and reverse directions.
 * 
 * Control logic:
 * - If pwmForward > 0: Enable forward direction
 * - If pwmReverse > 0: Enable reverse direction
 * - If both 0: Motor coasts (no holding torque)
 * 
 * Enable pins are typically held HIGH during operation (active-high logic).
 */
struct Bts7960Pins {
  uint8_t pwmForward;   ///< PWM pin for forward direction (0-255)
  uint8_t pwmReverse;   ///< PWM pin for reverse direction (0-255)
  uint8_t enableForward; ///< Enable pin for forward direction (active-high)
  uint8_t enableReverse; ///< Enable pin for reverse direction (active-high)
};

/**
 * @brief Motor driver class for BTS7960 dual-channel H-bridge.
 * 
 * Inherits from MotorBase and implements hardware-specific PWM control.
 * Used for RA (Right Ascension) and DEC (Declination) motors.
 * 
 * Control Approach:
 * - Uses independent PWM channels for forward and reverse
 * - Positive PWM applies to forward channel, reverse channel at 0
 * - Negative PWM applies to reverse channel, forward channel at 0
 * - Zero PWM stops motor (coasts)
 * 
 * Advantages:
 * - Independent control of both directions allows smooth transitions
 * - Built-in current limiting and thermal protection
 * - Efficient two-channel PWM approach
 * 
 * Hardware Details:
 * - DEC motor uses pins 6-7 (PWM) and 22-23 (enable)
 * - RA motor uses pins 8-9 (PWM) and 24-25 (enable)
 */
class BTS7960Motor : public MotorBase {
 public:
  /**
   * @brief Constructor for BTS7960Motor.
   * @param name Motor name for debugging ("RA" or "DEC")
   * @param encoderId Which encoder input to use
   * @param pins Pin configuration for this motor instance
   * @param pid PID controller tuning
   * @param runtime Control loop parameters
   * @param limits Limit switch configuration
   * @param calibration Calibration process parameters
   * @param precision Precision positioning parameters
   */
  BTS7960Motor(const char* name,
               MotorId encoderId,
               const Bts7960Pins& pins,
               const PidConfig& pid,
               const MotorRuntimeConfig& runtime,
               const LimitConfig& limits,
               const CalibrationConfig& calibration,
               const PrecisionConfig& precision);

 protected:
  /**
   * @brief Configure BTS7960 control pins as outputs.
   * Initializes PWM pins and enable pins, zeroes all outputs.
   */
  void setupDriverPins() override;

  /**
   * @brief Apply signed PWM to the BTS7960 driver.
   * @param signedPwm Signed PWM value (-255 to +255)
   *        Positive: forward direction
   *        Negative: reverse direction
   *        Zero: motor stopped
   * 
   * Maps the signed value to independent forward/reverse PWM channels.
   */
  void applyDriverPwm(int16_t signedPwm) override;

 private:
  Bts7960Pins m_pins;  ///< Pin configuration for this motor instance
};

}  // namespace refactored

#endif  // BTS7960_MOTOR_HPP
