#ifndef MD13S_MOTOR_HPP
#define MD13S_MOTOR_HPP

#include "motor_base.hpp"

namespace refactored {

struct Md13sPins {
  uint8_t pwm;
  uint8_t dir;
  bool invertCommandSign;
};

struct SpeedPidConfig {
  float kp;
  float ki;
  float kd;
  float integralMin;
  float integralMax;
  float positionAssistKp;
  uint8_t slowdownWindowTicks;
  float minSpeedTicksPerSec;
};

class MD13SMotor : public MotorBase {
 public:
  MD13SMotor(const char* name,
             MotorId encoderId,
             const Md13sPins& pins,
             const PidConfig& positionPid,
             const SpeedPidConfig& speedPid,
             const MotorRuntimeConfig& runtime,
             const LimitConfig& limits,
             const CalibrationConfig& calibration,
             const PrecisionConfig& precision);

  bool commandSpeedTicksPerSecond(long targetTicks, float ticksPerSecond) override;

 protected:
  void setupDriverPins() override;
  void applyDriverPwm(int16_t signedPwm) override;
  int16_t computeControlPwm(long errorTicks,
                            float dtSeconds,
                            long currentTicks,
                            unsigned long nowMs) override;

 private:
  Md13sPins m_pins;
  SpeedPidConfig m_speedPid;

  float m_requestedSpeedTicksPerSec;
  float m_speedIntegral;
  float m_prevSpeedError;
  float m_profileTick;
  long m_prevSpeedTick;
};

}  // namespace refactored

#endif  // MD13S_MOTOR_HPP
