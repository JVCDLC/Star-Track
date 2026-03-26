#ifndef BTS7960_MOTOR_HPP
#define BTS7960_MOTOR_HPP

#include "motor_base.hpp"

namespace refactored {

struct Bts7960Pins {
  uint8_t pwmForward;
  uint8_t pwmReverse;
  uint8_t enableForward;
  uint8_t enableReverse;
};

class BTS7960Motor : public MotorBase {
 public:
  BTS7960Motor(const char* name,
               MotorId encoderId,
               const Bts7960Pins& pins,
               const PidConfig& pid,
               const MotorRuntimeConfig& runtime,
               const LimitConfig& limits,
               const CalibrationConfig& calibration,
               const PrecisionConfig& precision);

 protected:
  void setupDriverPins() override;
  void applyDriverPwm(int16_t signedPwm) override;

 private:
  Bts7960Pins m_pins;
};

}  // namespace refactored

#endif  // BTS7960_MOTOR_HPP
