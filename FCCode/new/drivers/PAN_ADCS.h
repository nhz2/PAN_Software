#ifndef PAN_ADCS_DRIVER_H_
#define PAN_ADCS_DRIVER_H_

#include "PANDriver.h"

using namespace PAN::Devices;

class ADCS : public PANDriver {
  public:
    bool setup();
    bool isResponding();
};

#endif