#include <Arduino.h>
#include "switchs.hpp"
#include "constants.hpp"

// save initial state of the ports to detect changes
volatile uint8_t last_portK;

volatile bool switch1_triggered = false;
volatile bool switch2_triggered = false;
volatile bool switch3_triggered = false;
volatile bool switch4_triggered = false;
volatile bool switch5_triggered = false;
volatile bool switch6_triggered = false;
volatile bool switch7_triggered = false;
volatile bool switch8_triggered = false;
volatile bool switch9_triggered = false;
volatile bool switch10_triggered = false;

void switches_init()
{
    // reset flags
    switch1_triggered = true;
    switch2_triggered = true;
    switch3_triggered = true;
    switch4_triggered = true;
    switch5_triggered = true;
    switch6_triggered = true;
    switch7_triggered = true;
    switch8_triggered = true;
    switch9_triggered = true;
    switch10_triggered = true;

    pinMode(SWITCH_1, INPUT_PULLUP);
    pinMode(SWITCH_2, INPUT_PULLUP);
    pinMode(SWITCH_3, INPUT_PULLUP);
    pinMode(SWITCH_4, INPUT_PULLUP);
    pinMode(SWITCH_5, INPUT_PULLUP);
    pinMode(SWITCH_6, INPUT_PULLUP);
    pinMode(SWITCH_7, INPUT_PULLUP);
    pinMode(SWITCH_8, INPUT_PULLUP);
    pinMode(SWITCH_9, INPUT_PULLUP);
    pinMode(SWITCH_10, INPUT_PULLUP);

    // save initial state
    last_portK = PINK; // A8-A15

    // enable interrupt on port K
    PCICR |= (1 << PCIE2);
    PCMSK2 |= 0xFF;

    // enable interrupt on port B
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT6) | (1 << PCINT7);
}

// interrupt for A8-A15
ISR(PCINT2_vect)
{
    uint8_t current = PINK;
    uint8_t changed = current ^ last_portK;
    last_portK = current;

    if (changed & (1 << 0)) switch1_triggered = ((current & (1 << 0)) == 0);
    if (changed & (1 << 1)) switch2_triggered = ((current & (1 << 1)) == 0);
    if (changed & (1 << 2)) switch3_triggered = ((current & (1 << 2)) == 0);
    if (changed & (1 << 3)) switch4_triggered = ((current & (1 << 3)) == 0);
    if (changed & (1 << 4)) switch5_triggered = ((current & (1 << 4)) == 0);
    if (changed & (1 << 5)) switch6_triggered = ((current & (1 << 5)) == 0);
    if (changed & (1 << 6)) switch7_triggered = ((current & (1 << 6)) == 0);
    if (changed & (1 << 7)) switch8_triggered = ((current & (1 << 7)) == 0);
}
