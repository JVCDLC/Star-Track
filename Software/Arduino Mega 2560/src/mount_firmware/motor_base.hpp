#ifndef MOTOR_BASE_HPP
#define MOTOR_BASE_HPP

#include <Arduino.h>

#include "interrupt_hub.hpp"

namespace refactored {

enum class MotorState : uint8_t {
  UNINITIALIZED = 0,          ///< Initial state before begin() is called
  BACKOFF_FROM_LIMIT,        ///< Moving away from a limit switch that was triggered at startup
  CALIBRATING_MIN,            ///< Fast seeking toward minimum limit switch
  BACKOFF_AFTER_MIN,          ///< Backing off after hitting minimum limit
  CALIBRATING_MIN_VERIFY,     ///< Slow verification pass for minimum limit
  BACKOFF_AFTER_MIN_VERIFY,   ///< Backing off after verification of minimum limit
  CALIBRATING_MAX,            ///< Fast seeking toward maximum limit switch
  BACKOFF_AFTER_MAX,          ///< Backing off after hitting maximum limit
  CALIBRATING_MAX_VERIFY,     ///< Slow verification pass for maximum limit
  BACKOFF_AFTER_MAX_VERIFY,   ///< Backing off after verification of maximum limit
  CENTERING,                  ///< Moving to geometric center between limits
  STICTION_SCAN_POS,          ///< Scanning for minimum PWM needed for positive direction motion
  STICTION_SCAN_NEG,          ///< Scanning for minimum PWM needed for negative direction motion
  CALIB_RETURN_TO_ZERO,       ///< Returning to logical zero position after calibration
  READY,                      ///< Ready for commands, at rest
  MOVING,                     ///< Executing position control command
  SPEED_MOVING,               ///< Executing speed control command (focus only)
  MICRO_CORRECTION,           ///< Fine-tuning position after overshoot
  STOPPED,                    ///< Stopped but not at target (emergency stop)
  ERROR                       ///< Error state (calibration failed, timeout, etc.)
};

enum class MotionMode : uint8_t {
  POSITION = 0,
  SPEED = 1
};

struct PidConfig {
  float kp;
  float ki;
  float kd;
  float integralMin;  ///< Minimum allowed integral term (anti-windup)
  float integralMax;  ///< Maximum allowed integral term (anti-windup)
};

struct MotorRuntimeConfig {
  int16_t pwmMax;                ///< Maximum PWM value (0-255 for Arduino PWM)
  uint16_t controlPeriodMs;      ///< How often to run the control loop (in ms)
  uint16_t noMotionTimeoutMs;    ///< Timeout before considering motor stuck
  uint8_t atTargetToleranceTicks; ///< Position tolerance for considering target reached
};

struct LimitConfig {
  bool hasMinSwitch;     ///< Whether minimum limit switch is configured
  SwitchId minSwitch;    ///< Which switch ID is the minimum limit
  int8_t dirTowardMin;   ///< Motor direction (+1/-1) that moves toward minimum limit

  bool hasMaxSwitch;     ///< Whether maximum limit switch is configured
  SwitchId maxSwitch;    ///< Which switch ID is the maximum limit
  int8_t dirTowardMax;   ///< Motor direction (+1/-1) that moves toward maximum limit

  long fallbackSpanTicks; ///< Fallback range if switches fail (used for safety)
};

struct CalibrationConfig {
  uint8_t fastSeekPwm;           ///< PWM for fast seeking to limits (high speed)
  uint8_t verifySeekPwm;         ///< PWM for slow verification pass (precision)
  uint8_t backoffPwm;            ///< PWM for backing off from switches
  uint16_t backoffDurationMs;    ///< How long to back off after hitting switch
  uint16_t releaseStableMs;      ///< Time to wait for switch to stabilize after release
  uint8_t verifyDeltaToleranceTicks; ///< Position tolerance for verification passes
  bool singleSwitchHomeOnly;     ///< True for focus (only min switch), false for RA/DEC
  uint8_t centerToleranceTicks;  ///< Position tolerance for centering
  uint16_t centerSettleMs;       ///< Time to settle at center before declaring done

  uint8_t stictionScanStartPwm;  ///< Starting PWM for stiction binary search
  uint8_t stictionScanStepPwm;   ///< PWM increment for stiction scanning
  uint8_t stictionScanMaxPwm;    ///< Maximum PWM to try in stiction scan
  uint16_t stictionSampleWindowMs; ///< Time to sample motion at each PWM level
  uint8_t stictionDetectTicks;   ///< Minimum ticks of motion to detect as "moving"
};

struct PrecisionConfig {
  uint8_t overshootWindowTicks;    ///< Error window for triggering micro-correction
  uint16_t reverseKickMs;          ///< Duration of reverse kick in micro-correction
  uint8_t reverseKickExtraPwm;     ///< Extra PWM added during reverse kick
  uint16_t adaptiveRaiseDelayMs;   ///< Delay before raising stiction floor when stuck
  uint8_t adaptiveRaiseStep;       ///< How much to increase stiction floor
  uint8_t adaptiveDropStep;        ///< How much to decrease stiction floor when overshooting
};

/**
 * @brief Abstract base class for motor control.
 * 
 * This class implements the core motor control logic including:
 * - PID-based position control
 * - Multi-pass limit switch calibration
 * - Stiction compensation and adaptive floor adjustment
 * - Micro-correction for precision positioning
 * - Safety timeouts and error handling
 * 
 * Derived classes (BTS7960Motor, MD13SMotor) implement hardware-specific driver control.
 * 
 * Key design principles:
 * - Non-blocking: update() must be called frequently but returns quickly
 * - State machine: Complex operations (calibration) use non-blocking state progression
 * - Safety first: Multiple layers of limit checking and timeout protection
 */
class MotorBase {
 public:
  MotorBase(const char* name,                     // Motor name
            MotorId encoderId,                    // Which encoder this motor reads from
            const PidConfig& pid,                 // PID configuration for position control
            const MotorRuntimeConfig& runtime,    // Runtime parameters (PWM limits, control period, timeouts)
            const LimitConfig& limits,            // Limit switch configuration
            const CalibrationConfig& calibration, // Calibration procedure parameters (speeds, timings, tolerances)
            const PrecisionConfig& precision);    // Precision control parameters (overshoot handling, adaptive stiction)

  virtual ~MotorBase() {} // Virtual destructor for base class

  void begin();
  void update(unsigned long nowMs);

  void requestCalibration();
  bool canAcceptCommand(long targetTicks) const;
  bool commandPositionTicks(long targetTicks);
  virtual bool commandSpeedTicksPerSecond(long targetTicks, float ticksPerSecond);
  void emergencyStopAndHold();
  bool setPositionPidGains(float kp, float ki, float kd);

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

  void debugApplyPwm(int16_t pwm) { applyDriverPwm(pwm); }
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
  virtual void onTargetReached(unsigned long nowMs, long currentTicks) {
    (void)nowMs;
    (void)currentTicks;
  }

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

  PidConfig m_pid;
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
