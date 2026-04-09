#include <Arduino.h>
#include <unity.h>

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)
#include "../src/mount_firmware/serial_protocol.hpp"

using namespace refactored;

void test_parse_absolute_move_dec() {
  SerialProtocol parser;
  SerialCommand cmd;
  char line[] = "1,DEC,-2500";
  TEST_ASSERT_TRUE(parser.parseFromCString(line, cmd));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandAction::MOVE_ABSOLUTE), static_cast<uint8_t>(cmd.action));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotorSelector::MOTOR_DEC), static_cast<uint8_t>(cmd.motor));
  TEST_ASSERT_EQUAL(-2500, cmd.targetTicks);
}

void test_parse_focus_speed_move() {
  SerialProtocol parser;
  SerialCommand cmd;
  char line[] = "2,FOC,105,10.5";
  TEST_ASSERT_TRUE(parser.parseFromCString(line, cmd));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandAction::MOVE_WITH_SPEED), static_cast<uint8_t>(cmd.action));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotorSelector::MOTOR_FOC), static_cast<uint8_t>(cmd.motor));
  TEST_ASSERT_EQUAL(105, cmd.targetTicks);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.5f, cmd.speedTicksPerSecond);
}

void test_parse_single_motor_calibration() {
  SerialProtocol parser;
  SerialCommand cmd;
  char line[] = "4,RA";
  TEST_ASSERT_TRUE(parser.parseFromCString(line, cmd));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandAction::CALIBRATE_ONE), static_cast<uint8_t>(cmd.action));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotorSelector::MOTOR_RA), static_cast<uint8_t>(cmd.motor));
}

void test_parse_motion_self_test() {
  SerialProtocol parser;
  SerialCommand cmd;
  char line[] = "5,DEC,1200";
  TEST_ASSERT_TRUE(parser.parseFromCString(line, cmd));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandAction::MOTION_SELF_TEST), static_cast<uint8_t>(cmd.action));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotorSelector::MOTOR_DEC), static_cast<uint8_t>(cmd.motor));
  TEST_ASSERT_EQUAL(1200, cmd.auxTicks);
}

void test_parse_debug_focus() {
  SerialProtocol parser;
  SerialCommand cmd;
  char line[] = "6,FOC";
  TEST_ASSERT_TRUE(parser.parseFromCString(line, cmd));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandAction::DEBUG_FOCUS), static_cast<uint8_t>(cmd.action));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotorSelector::MOTOR_FOC), static_cast<uint8_t>(cmd.motor));
}

void test_parse_set_pid() {
  SerialProtocol parser;
  SerialCommand cmd;
  char line[] = "7,FOC,SPD,2.8,0.2,0.05";
  TEST_ASSERT_TRUE(parser.parseFromCString(line, cmd));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(CommandAction::SET_PID), static_cast<uint8_t>(cmd.action));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(MotorSelector::MOTOR_FOC), static_cast<uint8_t>(cmd.motor));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(PidSelector::SPEED), static_cast<uint8_t>(cmd.pidSelector));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.8f, cmd.pidKp);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2f, cmd.pidKi);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, cmd.pidKd);
}

#else
void test_refactored_not_enabled() {
  TEST_PASS();
}
#endif

void setup() {
  delay(1000);
  UNITY_BEGIN();
#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)
  RUN_TEST(test_parse_absolute_move_dec);
  RUN_TEST(test_parse_focus_speed_move);
  RUN_TEST(test_parse_single_motor_calibration);
  RUN_TEST(test_parse_motion_self_test);
  RUN_TEST(test_parse_debug_focus);
  RUN_TEST(test_parse_set_pid);
#else
  RUN_TEST(test_refactored_not_enabled);
#endif
  UNITY_END();
}

void loop() {}
