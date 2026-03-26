#include <Arduino.h>

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include "refactored/mount_controller.hpp"

static refactored::MountController g_mount;

void setup() {
  g_mount.begin();
}

void loop() {
  g_mount.update();
}

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
