#ifndef GOMSPACE_DRIVER_H_
#define GOMSPACE_DRIVER_H_

#include "PANDriver.h"

using namespace PAN::Devices;

class Gomspace : public PANDriver {
  public:
    bool setup();
    bool isResponding();
};

#endif