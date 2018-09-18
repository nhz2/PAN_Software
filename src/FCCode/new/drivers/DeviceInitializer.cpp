#include "../state/MasterStateHolder.h"
#include "DeviceInitializer.h"
#include "../state/HardwareAvailabilityTable.h"

using namespace PAN::Devices;

void initialize_all() {
    HAVT havt = msh.get_havt();
    for(uint8_t i = 0; i < havt.get_device_list().length(); i++) {

    }
}