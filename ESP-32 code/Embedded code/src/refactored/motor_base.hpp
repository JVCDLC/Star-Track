#ifndef MOTOR_BASE_HPP
#define MOTOR_BASE_HPP

#include <Arduino.h>

#include "interrupt_hub.hpp"

namespace refactored {

enum class MotorState : uint8_t {
  UNINITIALIZED = 0,
  BACKOFF_FROM_LIMIT,
  CALIBRATING_MIN,
  BACKOFF_AFTER_MIN,
  CALIBRATING_MIN_VERIFY,
  BACKOFF_AFTER_MIN_VERIFY,
  CALIBRATING_MAX,
  BACKOFF_AFTER_MAX,
  CALIBRATING_MAX_VERIFY,
  BACKOFF_AFTER_MAX_VERIFY,
  CENTERING,
  STICTION_SCAN_POS,
  STICTION_SCAN_NEG,
  CALIB_RETURN_TO_ZERO,
  READY,
  MOVING,
  SPEED_MOVING,
  MICRO_CORRECTION,
  STOPPED,
  ERROR
};

enum class MotionMode : uint8_t {
  POSITION = 0,
  SPEED = 1
};

struct PidConfig {
  float kp;
  float ki;
  float kd;
  float integralMin;
  float integralMax;
};

struct MotorRuntimeConfig {
  int16_t pwmMax;
  uint16_t controlPeriodMs;
  uint16_t noMotionTimeoutMs;
  uint8_t atTargetToleranceTicks;
};

struct LimitConfig {
  bool hasMinSwitch;
  SwitchId minSwitch;
  int8_t dirTowardMin;  // +1 or -1 in motor command sign convention

  bool hasMaxSwitch;
  SwitchId maxSwitch;
  int8_t dirTowardMax;  // +1 or -1 in motor command sign convention

  long fallbackSpanTicks;
};

struct CalibrationConfig {
  uint8_t fastSeekPwm;
  uint8_t verifySeekPwm;
  uint8_t backoffPwm;
  uint16_t backoffDurationMs;
  uint16_t releaseStableMs;
  uint8_t verifyDeltaToleranceTicks;
  bool singleSwitchHomeOnly;
  uint8_t centerToleranceTicks;
  uint16_t centerSettleMs;

  uint8_t stictionScanStartPwm;
  uint8_t stictionScanStepPwm;
  uint8_t stictionScanMaxPwm;
  uint16_t stictionSampleWindowMs;
  uint8_t stictionDetectTicks;
};

struct PrecisionConfig {
  uint8_t overshootWindowTicks;
  uint16_t reverseKickMs;
  uint8_t reverseKickExtraPwm;
  uint16_t adaptiveRaiseDelayMs;
  uint8_t adaptiveRaiseStep;
  uint8_t adaptiveDropStep;
};

class MotorBase {
 public:
  // Core contract:
  // - All control uses encoder ticks only (no degree conversion in runtime loop).
  // - update() is strictly non-blocking and must be called at high frequency.
  // - each motor owns its own PID, limits, calibration flow and safety timers.
  MotorBase(const char* name,
            MotorId encoderId,
            const PidConfig& pid,
            const MotorRuntimeConfig& runtime,
            const LimitConfig& limits,
            const CalibrationConfig& calibration,
            const PrecisionConfig& precision);

  virtual ~MotorBase() {}

  void begin();
  void update(unsigned long nowMs);

  void requestCalibration();
  bool commandPositionTicks(long targetTicks);
  virtual bool commandSpeedTicksPerSecond(long targetTicks, float ticksPerSecond);
  void emergencyStopAndHold();

  long readTicks() const;
  void writeTicks(long ticks);

  long readTargetTicks() const { return m_targetTicks; }
  long readMinTick() const { return m_limitMinTick; }
  long readMaxTick() const { return m_limitMaxTick; }
  uint8_t readMinPwmPos() const { return m_adaptiveMinPwmPos; }
  uint8_t readMinPwmNeg() const { return m_adaptiveMinPwmNeg; }

  MotorState readState() const { return m_state; }
  const char* readName() const { return m_name; }
  bool isReady() const { return m_state == MotorState::READY; }
  bool isError() const { return m_state == MotorState::ERROR; }
  bool isCalibrating() const;

 protected:
  virtual void setupDriverPins() = 0;
  virtual void applyDriverPwm(int16_t signedPwm) = 0;

  // Derived classes can override for mode-specific control (focus speed mode).
  virtual int16_t computeControlPwm(long errorTicks,
                                    float dtSeconds,
                                    long currentTicks,
                                    unsigned long nowMs);

  MotionMode readMotionMode() const { return m_motionMode; }
  long readCurrentTargetForControl() const { return m_targetTicks; }

  void setState(MotorState state) { m_state = state; }
  void resetController();
  void forceReadyAtCurrentPosition();

  uint8_t readAdaptiveMinPwmForDir(int8_t direction) const;
  bool directionBlockedByLimit(int8_t direction) const;

  static int8_t signOf(long value);
  static long absLong(long value);

  const char* m_name;
  const MotorId m_encoderId;

  const PidConfig m_pid;
  const MotorRuntimeConfig m_runtime;
  const LimitConfig m_limits;
  const CalibrationConfig m_calibration;
  const PrecisionConfig m_precision;

  MotionMode m_motionMode;
  MotorState m_state;

  bool m_limitsKnown;
  long m_limitMinTick;
  long m_limitMaxTick;
  long m_targetTicks;

  float m_errorIntegral;
  long m_prevErrorTicks;

  unsigned long m_lastControlMs;
  unsigned long m_lastMotionMs;
  long m_lastObservedTick;
  int8_t m_lastAppliedDirection;

  uint8_t m_stictionMinPwmPos;
  uint8_t m_stictionMinPwmNeg;
  uint8_t m_adaptiveMinPwmPos;
  uint8_t m_adaptiveMinPwmNeg;

 private:
  void driveOpenLoopDirectional(int8_t direction, uint8_t pwmMagnitude);

  void updateCalibration(unsigned long nowMs);
  void updateMotion(unsigned long nowMs, float dtSeconds);

  void updateStictionScan(unsigned long nowMs, int8_t direction);
  void startNextStictionStep(unsigned long nowMs, int8_t direction);

  void enterError();
  void applyCommandedPwm(int16_t signedPwm, unsigned long nowMs, long currentTicks, long errorTicks);
  void beginMicroCorrection(unsigned long nowMs, long currentErrorTicks);
  void updateMicroCorrection(unsigned long nowMs, long currentErrorTicks, long currentTicks);
  bool anyConfiguredLimitTriggered() const;
  bool targetInsideKnownLimits(long targetTicks) const;

  // Non-blocking calibration scratch variables.
  long m_calCapturedMinTick;
  long m_calCapturedMaxTick;
  long m_calFirstHitTick;
  long m_calCenterTick;
  unsigned long m_calStateStartMs;
  unsigned long m_releaseStableStartMs;
  unsigned long m_centerStableStartMs;

  bool m_scanStepActive;
  uint8_t m_scanPwm;
  long m_scanStartTick;
  unsigned long m_scanStepStartMs;

  // Non-blocking overshoot correction scratch variables.
  unsigned long m_microStartMs;
  int8_t m_microDirection;
};

}  // namespace refactored

#endif  // MOTOR_BASE_HPP
