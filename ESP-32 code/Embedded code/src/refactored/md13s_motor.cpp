#include "md13s_motor.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include <math.h>

namespace refactored {

/**
 * @brief Constructor for MD13SMotor (PWM+DIR driver with speed control).
 * @param name Motor name (e.g., "Focus")
 * @param encoderId Encoder ID (e.g., MOTOR_FOCUS)
 * @param pins MD13S pins configuration (PWM, DIR, optional inversion)
 * @param positionPid PID tuning for position control during calibration
 * @param speedPid SpeedPidConfig with speed loop tuning and velocity profile settings
 * @param runtime Motion constraints (min/max speed, acceleration, timeout)
 * @param limits Limit switch configuration
 * @param calibration Calibration parameters
 * @param precision Micro-correction settings
 * 
 * Stores both position and speed PID configurations since the motor supports two modes:
 * - POSITION mode during calibration (inherited MotorBase PID control)
 * - SPEED mode during user-commanded focus moves (this class's dual-loop algorithm)
 * 
 * Speed control state initialized to zero:
 * - m_requestedSpeedTicksPerSec: Desired constant speed (set by commandSpeedTicksPerSecond)
 * - m_speedIntegral: Accumulated speed error for PID integral term
 * - m_prevSpeedError: Previous speed error for PID derivative term
 * - m_profileTick: Virtual position moving at constant speed (generates smooth reference)
 * - m_prevSpeedTick: Previous encoder position for velocity calculation
 */
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
      m_requestedSpeedTicksPerSec(0.0f),  // No speed requested initially
      m_speedIntegral(0.0f),               // Speed PID integral accumulator
      m_prevSpeedError(0.0f),              // Previous speed error for derivative
      m_profileTick(0.0f),                 // Virtual position tracking
      m_prevSpeedTick(0) {}                // Base position for velocity calc

/**
 * @brief Command motor to move at constant speed to a target position.
 * @param targetTicks Final position where motor should stop
 * @param ticksPerSecond Desired constant speed in encoder ticks/second
 * @return true if command accepted and speed control started
 * 
 * Validate command through position check (ensures target is in valid range),
 * then initiate dual-loop speed control:
 * 1. Validates speed is positive (negative used for direction only)
 * 2. Uses base class position validation to check target is in range
 * 3. Switches motion mode from POSITION to SPEED
 * 4. Resets speed PID state for clean start (zero integral and derivative)
 * 5. Initializes velocity profile at current position
 * 
 * After return, the motor will accelerate/decelerate smoothly to requested speed,
 * then maintain that speed while moving toward targetTicks, with automatic slowdown
 * near the target to prevent overshoot (configured in SpeedPidConfig.slowdownWindowTicks).
 */
bool MD13SMotor::commandSpeedTicksPerSecond(long targetTicks, float ticksPerSecond) {
  // Validate speed is positive (direction determined by target vs current)
  if (ticksPerSecond <= 0.0f) {
    return false;  // Speed must be positive
  }
  
  // Validate target position is in valid range (not beyond limits, not during calibration, etc.)
  if (!commandPositionTicks(targetTicks)) {
    return false;  // Position validation fails
  }

  // Switch to speed control mode
  m_requestedSpeedTicksPerSec = ticksPerSecond;  // Store target speed
  m_motionMode = MotionMode::SPEED;              // Switch motion mode
  m_state = MotorState::SPEED_MOVING;            // Set state machine to moving
  
  // Reset speed PID state for clean start without accumulated integral windup
  m_speedIntegral = 0.0f;
  m_prevSpeedError = 0.0f;
  
  // Initialize velocity profile: starts at current position and moves at constant speed
  m_profileTick = static_cast<float>(readTicks());  // Virtual position = actual position
  m_prevSpeedTick = readTicks();                    // Base for velocity delta calculation
  
  // Reset position PID controller (used for position assist component)
  resetController();
  
  return true;
}

/**
 * @brief Initialize MD13S driver pins as outputs and set to safe state.
 * 
 * Sets up the PWM+DIR control interface:
 * - DIR pin: Controls motor rotation direction (HIGH = forward, LOW = reverse)
 * - PWM pin: Controls motor speed (0-255, PWM duty cycle)
 * 
 * Both pins are set to 0/LOW on initialization, motor is stopped.
 */
void MD13SMotor::setupDriverPins() {
  // Set direction control pin to output mode
  pinMode(m_pins.dir, OUTPUT);
  // Set PWM speed control pin to output mode
  pinMode(m_pins.pwm, OUTPUT);
  // Initialize PWM to 0: motor stopped
  analogWrite(m_pins.pwm, 0);
}

/**
 * @brief Apply signed PWM command to MD13S motor driver.
 * @param signedPwm Signed PWM value (-255 to +255)
 *        - Positive: Forward direction at magnitude signedPwm
 *        - Negative: Reverse direction at magnitude |signedPwm|
 *        - Zero: Motor stopped
 * 
 * Command mapping (before inversion):
 * - signedPwm = +200: DIR=HIGH, PWM=200 (motor on, ~78% speed forward)
 * - signedPwm = -150: DIR=LOW, PWM=150 (motor on, ~59% speed reverse)
 * - signedPwm = 0: PWM=0 (motor coast to stop)
 * 
 * Optional inversion: If m_pins.invertCommandSign is true, negates the PWM
 * before applying. Useful if motor is mounted backwards or polarity is reversed.
 */
void MD13SMotor::applyDriverPwm(int16_t signedPwm) {
  // Optionally invert command sign (if motor is mounted backwards)
  int16_t pwm = signedPwm;
  if (m_pins.invertCommandSign) {
    pwm = static_cast<int16_t>(-pwm);
  }

  // Apply command to driver
  if (pwm > 0) {
    // Forward direction: set DIR HIGH, apply positive PWM magnitude
    digitalWrite(m_pins.dir, HIGH);
    analogWrite(m_pins.pwm, static_cast<uint8_t>(pwm));
  } else if (pwm < 0) {
    // Reverse direction: set DIR LOW, apply absolute value of PWM
    digitalWrite(m_pins.dir, LOW);
    analogWrite(m_pins.pwm, static_cast<uint8_t>(-pwm));
  } else {
    // Stopped: PWM to 0
    analogWrite(m_pins.pwm, 0);
  }
}

/**
 * @brief Compute control PWM using dual-loop speed control or base class position PID.
 * @param errorTicks Signed position error (targetTicks - currentTicks)
 * @param dtSeconds Time delta since previous call in seconds
 * @param currentTicks Current encoder position
 * @param nowMs Current timestamp (unused in this implementation)
 * @return Signed PWM command (-255 to +255)
 * 
 * ============================================================================
 * DUAL-LOOP SPEED CONTROL ALGORITHM
 * ============================================================================
 * 
 * This implementation provides smooth, load-responsive speed control by
 * combining two control loops:
 * 
 * 1. SPEED LOOP: Maintains constant velocity by measuring actual motor speed
 *    and adjusting PWM to match requested speed
 * 
 * 2. POSITION ASSIST LOOP: Keeps motor tracking a smooth reference trajectory
 *    that moves at the requested speed toward the target
 * 
 * The combination allows:
 * - Smooth acceleration (speed loop ramps to target speed)
 * - Responsive to load changes (speed error feedback)
 * - Accurate positioning (position assist prevents drift from profile)
 * - Smooth deceleration (automatic slowdown near target)
 * 
 * ============================================================================
 * STEP-BY-STEP EXECUTION
 * ============================================================================
 */
int16_t MD13SMotor::computeControlPwm(long errorTicks,
                                      float dtSeconds,
                                      long currentTicks,
                                      unsigned long nowMs) {
  (void)nowMs;  // Unused: not needed for this algorithm

  ////////////////////////////////////////////////////////////////////////////
  // STEP 1: Check mode - if in POSITION mode, use base class PID instead
  ////////////////////////////////////////////////////////////////////////////
  // During calibration, the motor uses POSITION mode (from MotorBase).
  // In SPEED mode (user command), we execute dual-loop algorithm.
  // Base class handles limit detection, timeout, emergency stop.
  if (readMotionMode() != MotionMode::SPEED) {
    // Not in speed control mode, use inherited position PID
    return MotorBase::computeControlPwm(errorTicks, dtSeconds, currentTicks, nowMs);
  }

  ////////////////////////////////////////////////////////////////////////////
  // STEP 2: Check if already at target
  ////////////////////////////////////////////////////////////////////////////
  // If no error (at target), stop everything cleanly
  if (errorTicks == 0) {
    m_speedIntegral = 0.0f;  // Reset PID state
    m_prevSpeedError = 0.0f;
    return 0;  // Stop motor
  }

  ////////////////////////////////////////////////////////////////////////////
  // STEP 3: Calculate desired speed including direction
  ////////////////////////////////////////////////////////////////////////////
  // Determine if moving toward positive or negative target
  const int8_t direction = signOf(errorTicks);  // Returns -1, 0, or +1
  // Apply direction to speed: positive if moving forward, negative if reverse
  float desiredSpeed = direction * m_requestedSpeedTicksPerSec;

  ////////////////////////////////////////////////////////////////////////////
  // STEP 4: Slowdown - reduce speed near target to prevent overshoot
  ////////////////////////////////////////////////////////////////////////////
  // SpeedPidConfig.slowdownWindowTicks: distance at which to start decelerating
  // Example: slowdownWindowTicks=1000, requestedSpeed=500 ticks/sec
  //   - At distance=1000: desiredSpeed = 500 ticks/sec (full speed)
  //   - At distance=500: desiredSpeed = 250 ticks/sec (half speed)
  //   - At distance=100: desiredSpeed ≈ 50 ticks/sec (1/10 speed)
  const long distance = absLong(errorTicks);  // Absolute distance to target
  if (distance < m_speedPid.slowdownWindowTicks && m_speedPid.slowdownWindowTicks > 0) {
    // Linear ramp: speed proportional to remaining distance
    desiredSpeed *= (static_cast<float>(distance) / static_cast<float>(m_speedPid.slowdownWindowTicks));
    // Floor speed: maintain minimum to overcome stiction even when very close
    if (fabsf(desiredSpeed) < m_speedPid.minSpeedTicksPerSec) {
      desiredSpeed = direction * m_speedPid.minSpeedTicksPerSec;
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // STEP 5: Generate velocity profile - smooth reference trajectory
  ////////////////////////////////////////////////////////////////////////////
  // m_profileTick advances at constant speed, creating a smooth path
  // Example: requestedSpeed=500 ticks/sec, dt=0.005sec
  //   - Each cycle: m_profileTick += 500 * 0.005 = 2.5 ticks
  //   - After 10 cycles: m_profileTick has advanced 25 ticks
  // This creates the "reference trajectory" for motor to follow
  m_profileTick += desiredSpeed * dtSeconds;  // Advance profile
  
  // Prevent profile from overshooting target
  if ((direction > 0 && m_profileTick > static_cast<float>(m_targetTicks)) ||
      (direction < 0 && m_profileTick < static_cast<float>(m_targetTicks))) {
    m_profileTick = static_cast<float>(m_targetTicks);  // Clamp to target
  }
  // Convert profile position to integer ticks for error calculation
  const long profileTargetTicks = static_cast<long>(lroundf(m_profileTick));
  const long profileErrorTicks = profileTargetTicks - currentTicks;  // How far is motor from profile?

  ////////////////////////////////////////////////////////////////////////////
  // STEP 6: Measure actual speed from encoder position changes
  ////////////////////////////////////////////////////////////////////////////
  // Speed = change in position / time elapsed
  // Example: currentTicks=1000, m_prevSpeedTick=995, dt=0.005
  //   - delta = 1000 - 995 = 5 ticks
  //   - actualSpeed = 5 / 0.005 = 1000 ticks/sec
  const float actualSpeed = static_cast<float>(currentTicks - m_prevSpeedTick) / dtSeconds;
  // How far off speed are we from target?
  const float speedError = desiredSpeed - actualSpeed;  // Positive = too slow, Negative = too fast
  // Store current position for next cycle's delta calculation
  m_prevSpeedTick = currentTicks;

  ////////////////////////////////////////////////////////////////////////////
  // STEP 7: Speed PID Control - adjust PWM to match desired speed
  ////////////////////////////////////////////////////////////////////////////
  // Standard PID formula: output = Kp*error + Ki*∫error*dt + Kd*d(error)/dt
  
  // PROPORTIONAL: Response proportional to how far off speed we are
  // speedOut += Kp * speedError;
  
  // INTEGRAL: Accumulated speed error (low speed for extended time → increase output)
  // Prevents steady-state error (e.g., if motor can never reach requested speed,
  // integral term gradually increases PWM to try harder)
  m_speedIntegral += speedError * dtSeconds;  // Accumulate error over time
  // Anti-windup: clamp integral to prevent excessive accumulation
  if (m_speedIntegral > m_speedPid.integralMax) m_speedIntegral = m_speedPid.integralMax;
  if (m_speedIntegral < m_speedPid.integralMin) m_speedIntegral = m_speedPid.integralMin;
  // speedOut += Ki * m_speedIntegral;
  
  // DERIVATIVE: Rate of change of error (smooths response, reduces overshoot)
  // If speed error is getting smaller, derivative term backs off
  const float speedDerivative = (speedError - m_prevSpeedError) / dtSeconds;
  m_prevSpeedError = speedError;  // Store for next cycle
  // speedOut += Kd * speedDerivative;
  
  // Combine all PID terms
  const float speedOut = (m_speedPid.kp * speedError) +
                         (m_speedPid.ki * m_speedIntegral) +
                         (m_speedPid.kd * speedDerivative);

  ////////////////////////////////////////////////////////////////////////////
  // STEP 8: Position assist - keep motor on the velocity profile
  ////////////////////////////////////////////////////////////////////////////
  // If motor lags the profile, position assist pulls harder
  // If motor is ahead of profile, position assist backs off
  // This prevents drift while maintaining constant speed
  // Example: speedOut=100, profileError=10 ticks, positionAssistKp=0.5
  //   - positionAssist = 0.5 * 10 = 5
  //   - combined = 100 + 5 = 105 (extra PWM to catch up to profile)
  const float positionAssist = m_speedPid.positionAssistKp * static_cast<float>(profileErrorTicks);
  float combined = speedOut + positionAssist;

  ////////////////////////////////////////////////////////////////////////////
  // STEP 9: Add stiction floor and constrain to valid PWM range
  ////////////////////////////////////////////////////////////////////////////
  // Stiction floor: minimum PWM needed to overcome motor friction
  // Without this, motor won't move when speedOut is below stiction threshold
  const int16_t stiction = static_cast<int16_t>(readAdaptiveMinPwmForDir(direction));
  // Combine with stiction floor (applied in direction of motion)
  int32_t pwm = static_cast<int32_t>(lroundf(combined)) + (direction * stiction);
  
  // Ensure we meet or exceed stiction floor (can't go slower than needed to move)
  if (direction > 0 && pwm < stiction) pwm = stiction;      // Forward motion
  if (direction < 0 && pwm > -stiction) pwm = -stiction;    // Reverse motion

  // Clamp to valid PWM output range and return
  return static_cast<int16_t>(pwm);
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
