#include "md13s_motor.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include <math.h>

namespace refactored {

MD13SMotor::MD13SMotor(const char* name,
                       MotorId encoderId,
                       const Md13sPins& pins,
                       const PidConfig& positionPid,
                       const SpeedPidConfig& speedPid,
                       const MotorRuntimeConfig& runtime,
                       const LimitConfig& limits,
                       const CalibrationConfig& calibration,
                       const PrecisionConfig& precision)
    : MotorBase(name, encoderId, positionPid, runtime, limits, calibration, precision),
      m_pins(pins),
      m_speedPid(speedPid),
      m_requestedSpeedTicksPerSec(0.0f),
      m_speedIntegral(0.0f),
      m_prevSpeedError(0.0f),
      m_profileTick(0.0f),
      m_prevSpeedTick(0) {}

bool MD13SMotor::commandSpeedTicksPerSecond(long targetTicks, float ticksPerSecond) {
  if (ticksPerSecond <= 0.0f) {
    return false;
  }
  if (!commandPositionTicks(targetTicks)) {
    return false;
  }

  m_requestedSpeedTicksPerSec = ticksPerSecond;
  m_motionMode = MotionMode::SPEED;
  m_state = MotorState::SPEED_MOVING;
  m_speedIntegral = 0.0f;
  m_prevSpeedError = 0.0f;
  m_profileTick = static_cast<float>(readTicks());
  m_prevSpeedTick = readTicks();
  resetController();
  return true;
}

void MD13SMotor::setupDriverPins() {
  pinMode(m_pins.dir, OUTPUT);
  pinMode(m_pins.pwm, OUTPUT);
  analogWrite(m_pins.pwm, 0);
}

void MD13SMotor::applyDriverPwm(int16_t signedPwm) {
  int16_t pwm = signedPwm;
  if (m_pins.invertCommandSign) {
    pwm = static_cast<int16_t>(-pwm);
  }

  if (pwm > 0) {
    digitalWrite(m_pins.dir, HIGH);
    analogWrite(m_pins.pwm, static_cast<uint8_t>(pwm));
  } else if (pwm < 0) {
    digitalWrite(m_pins.dir, LOW);
    analogWrite(m_pins.pwm, static_cast<uint8_t>(-pwm));
  } else {
    analogWrite(m_pins.pwm, 0);
  }
}

int16_t MD13SMotor::computeControlPwm(long errorTicks,
                                      float dtSeconds,
                                      long currentTicks,
                                      unsigned long nowMs) {
  (void)nowMs;

  if (readMotionMode() != MotionMode::SPEED) {
    return MotorBase::computeControlPwm(errorTicks, dtSeconds, currentTicks, nowMs);
  }

  if (errorTicks == 0) {
    m_speedIntegral = 0.0f;
    m_prevSpeedError = 0.0f;
    return 0;
  }

  const int8_t direction = signOf(errorTicks);
  float desiredSpeed = direction * m_requestedSpeedTicksPerSec;

  const long distance = absLong(errorTicks);
  if (distance < m_speedPid.slowdownWindowTicks && m_speedPid.slowdownWindowTicks > 0) {
    desiredSpeed *= (static_cast<float>(distance) / static_cast<float>(m_speedPid.slowdownWindowTicks));
    if (fabsf(desiredSpeed) < m_speedPid.minSpeedTicksPerSec) {
      desiredSpeed = direction * m_speedPid.minSpeedTicksPerSec;
    }
  }

  // Virtual profile target with linear ticks/sec progression.
  m_profileTick += desiredSpeed * dtSeconds;
  if ((direction > 0 && m_profileTick > static_cast<float>(m_targetTicks)) ||
      (direction < 0 && m_profileTick < static_cast<float>(m_targetTicks))) {
    m_profileTick = static_cast<float>(m_targetTicks);
  }
  const long profileTargetTicks = static_cast<long>(lroundf(m_profileTick));
  const long profileErrorTicks = profileTargetTicks - currentTicks;

  const float actualSpeed = static_cast<float>(currentTicks - m_prevSpeedTick) / dtSeconds;
  const float speedError = desiredSpeed - actualSpeed;
  m_prevSpeedTick = currentTicks;

  m_speedIntegral += speedError * dtSeconds;
  if (m_speedIntegral > m_speedPid.integralMax) m_speedIntegral = m_speedPid.integralMax;
  if (m_speedIntegral < m_speedPid.integralMin) m_speedIntegral = m_speedPid.integralMin;

  const float speedDerivative = (speedError - m_prevSpeedError) / dtSeconds;
  m_prevSpeedError = speedError;

  const float speedOut = (m_speedPid.kp * speedError) +
                         (m_speedPid.ki * m_speedIntegral) +
                         (m_speedPid.kd * speedDerivative);

  const float positionAssist = m_speedPid.positionAssistKp * static_cast<float>(profileErrorTicks);
  float combined = speedOut + positionAssist;

  const int16_t stiction = static_cast<int16_t>(readAdaptiveMinPwmForDir(direction));
  int32_t pwm = static_cast<int32_t>(lroundf(combined)) + (direction * stiction);
  if (direction > 0 && pwm < stiction) pwm = stiction;
  if (direction < 0 && pwm > -stiction) pwm = -stiction;

  return static_cast<int16_t>(pwm);
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
