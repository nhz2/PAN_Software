#include <string>
#include <iostream>
#include "HardwareAvailabilityTable.h"

using namespace PAN::Devices;

bool HardwareAvailabilityTable::set_device_responding(std::string device_name, bool is_responding)
{
    auto device = devices.find(device_name);
    if (device == devices.end())
        return false;
    havt.at(device_name).responding = is_responding;
    return true;
}

bool HardwareAvailabilityTable::set_device_functional(std::string device_name, bool is_functional)
{
    auto device = devices.find(device_name);
    if (device == devices.end())
        return false;
    havt.at(device_name).functional = is_functional;
    return true;
}

auto const &HardwareAvailabilityTable::get_device_hardware_state(std::string device_name)
{
    auto device = devices.find(device_name);
    if (device == devices.end())
        return false;
    return havt.at(device_name);
}

bool HardwareAvailabilityTable::get_device_functional(std::string device_name)
{
    auto device = devices.find(device_name);
    if (device == devices.end())
        return false;
    return havt.at(device_name).functional;
}

bool HardwareAvailabilityTable::get_device_responding(std::string device_name)
{
    auto device = devices.find(device_name);
    if (device == devices.end())
        return false;
    return havt.at(device_name).responding;
}

auto const &HardwareAvailabilityTable::get_device_list()
{
    return devices;
}