#ifndef MOUNT_CONTROLLER_HPP
#define MOUNT_CONTROLLER_HPP

#include "bts7960_motor.hpp"
#include "md13s_motor.hpp"
#include "serial_protocol.hpp"

namespace refactored {

/**
 * @brief Overall state of the telescope mount.
 * Aggregated state based on individual motor states.
 * Transitions are controlled by updateMountState().
 */
enum class MountState : uint8_t {
  UNINITIALIZED = 0,  ///< Mount not yet initialized
  CALIBRATING,        ///< At least one motor is calibrating
  READY,              ///< All motors calibrated and ready for commands
  TRACKING,           ///< At least one motor is actively moving/controlling
  ERROR               ///< One or more motors in error state
};

/**
 * @brief Orchestrator for the three-axis telescope mount.
 * 
 * MountController manages:
 * - Three motor instances (RA, DEC, Focus)
 * - Serial protocol parsing and command dispatch
 * - State machine coordination across motors
 * - Debug output and diagnostics
 * - Motion self-tests
 * 
 * Architecture:
 * - Receives ASCII serial commands via SerialProtocol
 * - Validates and routes commands to appropriate motors
 * - Each motor operates independently with its own state machine
 * - MountController aggregates motor states to determine overall mount state
 * - All motor control happens in non-blocking 5ms update cycles
 * 
 * Usage:
 * ```
 * MountController mount;
 * mount.begin();     // Initialize hardware and start calibration
 * while(1)           // Main loop
 *   mount.update();  // Poll serial, update motors, manage state
 * ```
 */
class MountController {
 public:
  /**
   * @brief Constructor for MountController.
   * Initializes all three motors with hardcoded configuration and parameters.
   * Does NOT install interrupts - call begin() for that.
   */
  MountController();

  /**
   * @brief Initialize hardware and start the mount.
   * - Installs interrupt handlers for encoder and switch inputs
   * - Configures PWM timers at 31.25 kHz (silent to human hearing)
   * - Initializes serial port at 115200 baud
   * - Begins parallel calibration of all motors
   * 
   * Should be called once at startup (in Arduino setup()).
   */
  void begin();

  /**
   * @brief Main update function - call frequently from main loop.
   * - Polls serial port for incoming commands
   * - Processes valid commands and routes to motors
   * - Calls update() on all three motors
   * - Manages motion self-tests
   * - Updates overall mount state based on motor states
   * - Emits debug output periodically
   * 
   * This function must be called in a tight loop (main Arduino loop()).
   * It is non-blocking and completes quickly.
   */
  void update();

  /**
   * @brief Get the current overall mount state.
   * @return Current MountState (UNINITIALIZED, CALIBRATING, READY, TRACKING, ERROR)
   */
  MountState readState() const { return m_state; }

 private:
  /**
   * @brief Select a motor by selector enum.
   * @param selector Which motor (RA, DEC, FOC)
   * @return Pointer to the motor, or nullptr if invalid selector
   */
  MotorBase* selectMotor(MotorSelector selector);

  /**
   * @brief Process a single parsed serial command.
   * @param cmd The parsed command structure
   * @return true if command executed successfully
   * 
   * Routes the command to appropriate motor(s) or performs global actions.
   */
  bool processCommand(const SerialCommand& cmd);

  /**
   * @brief Update overall mount state based on individual motor states.
   * Aggregates motor states into MountState (READY, TRACKING, CALIBRATING, ERROR).
   * Called once per update() cycle.
   */
  void updateMountState();

  /**
   * @brief Emit periodic debug output to serial.
   * @param nowMs Current timestamp
   * 
   * Prints motor states, positions, and diagnostic info every 6 seconds.
   * Avoids spam by only printing state changes.
   */
  void emitDebug(unsigned long nowMs);

  /**
   * @brief Update motion self-test state machine.
   * @param nowMs Current timestamp
   * 
   * Monitors self-test progress and determines pass/fail.
   */
  void updateMotionSelfTest(unsigned long nowMs);

  /**
   * @brief Request parallel calibration of all motors.
   * All three motors calibrate simultaneously in parallel.
   */
  void startParallelCalibration();

  /**
   * @brief Request calibration of a single motor.
   * @param selector Which motor to calibrate
   * @return true if calibration started
   */
  bool startSingleMotorCalibration(MotorSelector selector);

  /**
   * @brief Emergency stop all motors immediately.
   * Sets all motors to emergency hold state.
   */
  void emergencyStopAll();

  /**
   * @brief Start a movement self-test for diagnostics.
   * @param selector Which motor to test
   * @param requestedDeltaTicks Expected minimum movement
   * @return true if test started
   */
  bool startMotionSelfTest(MotorSelector selector, long requestedDeltaTicks);

  // Member variables - grouped by purpose
  SerialProtocol m_serial;         ///< Serial command parser
  
  // Three motor instances (hardcoded configuration)
  BTS7960Motor m_motorDec;         ///< Declination motor (elevation)
  BTS7960Motor m_motorRa;          ///< Right Ascension motor (azimuth)
  MD13SMotor m_motorFoc;           ///< Focus motor (variable speed)
  
  // Mount state tracking
  MountState m_state;              ///< Current overall mount state
  
  // Debug state (for avoiding spam)
  unsigned long m_lastDebugMs;                ///< Last time debug was printed
  MountState m_lastPrintedMountState;        ///< Previous mount state (detect changes)
  MotorState m_lastPrintedRaState;           ///< Previous RA state (detect changes)
  MotorState m_lastPrintedDecState;          ///< Previous DEC state (detect changes)
  MotorState m_lastPrintedFocState;          ///< Previous FOC state (detect changes)

  // Focus debug state
  bool m_debugFocusActive;                   ///< Focus debug mode active
  unsigned long m_debugStartMs;              ///< When debug started
  unsigned long m_lastDebugPrintMs;          ///< Last time debug info was printed

  /**
   * @brief Motion self-test state machine.
   * Tracks a diagnostic test that verifies motor motion.
   */
  struct MotionSelfTest {
    bool active;                ///< Test currently running
    MotorBase* motor;           ///< Pointer to motor being tested
    long startTicks;            ///< Starting encoder position
    long minExpectedDelta;      ///< Minimum expected tick movement
    unsigned long startMs;      ///< Test start timestamp
    unsigned long timeoutMs;    ///< Test timeout duration
  } m_selfTest;
};

}  // namespace refactored

#endif  // MOUNT_CONTROLLER_HPP
