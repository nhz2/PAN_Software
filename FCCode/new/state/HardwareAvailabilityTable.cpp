#include <iostream>
#include "HardwareAvailabilityTable.h"

using namespace PAN::Devices;

bool HAVT::set_device_responding(std::string device_name, bool is_responding)
{
    auto device = devices.find(device_name);
    if (device == devices.end())
        return false;
    havt.at(device_name).responding = is_responding;
    return true;
}

bool HAVT::set_device_functional(std::string device_name, bool is_functional)
{
    auto device = devices.find(device_name);
    if (device == devices.end())
        return false;
    havt.at(device_name).functional = is_functional;
    return true;
}

auto const &get_device_list()
{
    return devices;
}