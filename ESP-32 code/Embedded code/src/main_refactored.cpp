#include <Arduino.h>

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include "refactored/mount_controller.hpp"

static refactored::MountController g_mount;

#ifndef PIO_UNIT_TESTING
void setup() {
  g_mount.begin();
}

void loop() {
  g_mount.update();
}

#endif

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH