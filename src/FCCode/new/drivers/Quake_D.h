#ifndef QUAKE_DRIVER_H_
#define QUAKE_DRIVER_H_

#include "PANDriver.h"

using namespace PAN::Devices;

class Quake : public PANDriver {
  public:
    bool setup();
    bool isResponding();
};

#endif