#include <stdlib.h>

#include "GenerateDeviceName.h"

static const char * DeviceNames[] = {
    "shocked-swan",
    "lucky-marlin",
    "exotic-snipe",
    "printed-boar",
    "medical-camel",
};

static const uint8_t DeviceNamesCount = sizeof(DeviceNames) / sizeof(DeviceNames[0]);

char *GenerateDeviceName(void)
{
  uint8_t index = (rand() + rand()) % DeviceNamesCount;
  return DeviceNames[index];
}
