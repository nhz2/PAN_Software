#ifndef PIKSI_DRIVER_H_
#define PIKSI_DRIVER_H_

#include "PANDriver.h"

using namespace PAN::Devices;

class Piksi : public PANDriver {
  public:
    bool setup();
    bool isResponding();
};

#endif