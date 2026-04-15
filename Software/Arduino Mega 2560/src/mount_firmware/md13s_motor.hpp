#ifndef MD13S_MOTOR_HPP
#define MD13S_MOTOR_HPP

#include "motor_base.hpp"

namespace refactored {

/**
 * @brief Pin configuration for MD13S single-channel motor driver.
 * 
 * The MD13S uses a simple PWM + DIR interface:
 * - pwm: Speed control (0-255)
 * - dir: Direction control (HIGH/LOW)
 * 
 * This is more efficient than BTS7960 (single PWM + 1 bit vs. two independent PWMs).
 * Used for focus motor which benefits from smooth speed ramps.
 */
struct Md13sPins {
  uint8_t pwm;              ///< PWM pin for speed (0-255)
  uint8_t dir;              ///< Direction pin (HIGH=forward, LOW=reverse)
  bool invertCommandSign;   ///< If true, invert PWM sign before applying to driver
};

/**
 * @brief PID configuration specifically for speed control.
 * 
 * Used by MD13SMotor for dual-loop control:
 * - Outer loop (position): Generates target speed profile
 * - Inner loop (speed): Achieves actual speed
 */
struct SpeedPidConfig {
  float kp;                  ///< Proportional gain for speed PID
  float ki;                  ///< Integral gain for speed PID
  float kd;                  ///< Derivative gain for speed PID
  float integralMin;         ///< Anti-windup minimum for integral term
  float integralMax;         ///< Anti-windup maximum for integral term
  float positionAssistKp;    ///< Gain for position-assist term (keeps motor tracking profile)
  uint8_t slowdownWindowTicks; ///< Distance from target to start slowing down (ticks)
  float minSpeedTicksPerSec; ///< Minimum speed to prevent stiction
  uint8_t snapWindowTicks;   ///< Distance where control switches to position PID for precise lock
  uint16_t speedLogSamplePeriodMs; ///< Sampling period for speed trace log
};

/**
 * @brief Motor driver class for MD13S single-channel driver with speed control.
 * 
 * Inherits from MotorBase and implements:
 * - Hardware PWM + DIR control for focus motor
 * - Dual-loop speed control (position + speed)
 * - Velocity profiling with linear ramps
 * - Position-assist feedback for tracking
 * 
 * Speed Control Algorithm:
 * 1. Position loop generates desired velocity profile
 * 2. Speed loop measures actual velocity from encoder
 * 3. Speed error drives PWM to match desired speed
 * 4. Position assist keeps motor tracking profile
 * 5. Slowdown window reduces speed as target approaches
 * 
 * This approach enables smooth, controllable focus motion without jerking.
 */
class MD13SMotor : public MotorBase {
 public:
  /**
   * @brief Constructor for MD13SMotor.
   * @param name Motor name for debugging ("FOC" for focus)
   * @param encoderId Which encoder input to use
   * @param pins Pin configuration for this motor
   * @param positionPid PID tuning for position control (during calibration)
   * @param speedPid PID tuning for speed control (during speed moves)
   * @param runtime Control loop parameters
   * @param limits Limit switch configuration
   * @param calibration Calibration process parameters
   * @param precision Precision positioning parameters
   */
  MD13SMotor(const char* name,
             MotorId encoderId,
             const Md13sPins& pins,
             const PidConfig& positionPid,
             const SpeedPidConfig& speedPid,
             const MotorRuntimeConfig& runtime,
             const LimitConfig& limits,
             const CalibrationConfig& calibration,
             const PrecisionConfig& precision);

  /**
   * @brief Command motor to move to target position at specified speed.
   * @param targetTicks Absolute target position in encoder ticks
   * @param ticksPerSecond Desired speed in ticks per second
   * @return true if command accepted
   * 
   * Sets motion mode to SPEED and triggers dual-loop speed control.
   * Speed is gradually ramped based on distance to target (slowdown window).
   * 
   * Example:
   * - commandSpeedTicksPerSecond(2000, 100.0) moves to tick 2000 at 100 ticks/sec
   */
  bool commandSpeedTicksPerSecond(long targetTicks, float ticksPerSecond) override;
  bool setSpeedPidGains(float kp, float ki, float kd);

 protected:
  /**
   * @brief Set up MD13S driver pins as outputs.
   * Initializes PWM and DIR pins to safe state (stopped).
   */
  void setupDriverPins() override;

  /**
   * @brief Apply signed PWM to MD13S driver.
   * @param signedPwm Signed value (-255 to +255)
   * 
   * Implements PWM + DIR control:
   * - DIR pin set based on sign of PWM
   * - PWM magnitude applied to speed
   * - Magnitude handling: 0-255 always positive, sign determines direction
   */
  void applyDriverPwm(int16_t signedPwm) override;

  /**
   * @brief Compute control PWM using dual-loop speed control.
   * @param errorTicks Current position error
   * @param dtSeconds Time delta since last update
   * @param currentTicks Current encoder position
   * @param nowMs Current timestamp
   * @return Signed PWM to apply
   * 
   * When in SPEED mode, implements dual-loop control:
   * 1. Position loop generates linear velocity profile
   * 2. Speed loop measures actual velocity from encoder deltas
   * 3. Combines speed error and position assist for smooth control
   * 4. Returns PWM that drives motor to match desired speed
   * 
   * When in POSITION mode, delegates to base class PID.
   */
  int16_t computeControlPwm(long errorTicks,
                            float dtSeconds,
                            long currentTicks,
                            unsigned long nowMs) override;
  void onTargetReached(unsigned long nowMs, long currentTicks) override;

 private:
  Md13sPins m_pins;                      ///< Pin configuration
  SpeedPidConfig m_speedPid;             ///< Speed control PID tuning

  // Speed control state variables
  float m_requestedSpeedTicksPerSec;    ///< Target speed from command
  float m_speedIntegral;                ///< Speed PID integral accumulator
  float m_prevSpeedError;               ///< Previous speed error (for derivative)
  float m_filteredActualSpeed;          ///< Low-pass filtered measured speed for stable control
  float m_profileTick;                  ///< Virtual position on linear speed profile
  long m_prevSpeedTick;                 ///< Position from previous cycle (for velocity calc)

  struct SpeedLogSample {
    uint16_t tMs;                ///< Elapsed time since speed move start (ms)
    int16_t desiredTicksPerSec;  ///< Desired speed sample (ticks/sec)
    int16_t actualTicksPerSec;   ///< Measured speed sample (ticks/sec)
  };
  static const uint16_t kSpeedLogCapacity = 192;
  SpeedLogSample m_speedLog[kSpeedLogCapacity];
  uint16_t m_speedLogCount;
  bool m_speedLogActive;
  unsigned long m_speedLogStartMs;
  unsigned long m_speedLogLastSampleMs;
  void resetSpeedLog(unsigned long nowMs);
  void maybeAppendSpeedLog(unsigned long nowMs, float desiredTicksPerSec, float actualTicksPerSec);
  void dumpSpeedLog(unsigned long nowMs, long finalTicks);
};

}  // namespace refactored

#endif  // MD13S_MOTOR_HPP
