#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include "rr_hw_interface/aio/revpi_aio.hpp"

namespace rapyuta
{

RevPiAio::RevPiAio(const std::string& pin_str, const Type& type)
        : HwInterface(pin_str, type)
{
}

RevPiAio::~RevPiAio()
{
}

bool RevPiAio::init(RevPiAioBoardConfig& config)
{
    sprintf(_pin.pin_name, "%s", _pin_str.c_str());
    int ret = revpi_init(&_pin);
    if (ret < 0) {
        ROS_ERROR("RevPiAio request for pin %s failed", _pin_str.c_str());
        return false;
    }

    return true;
}

void RevPiAio::set(int input)
{
    if (_type == Type::RR_HW_INTERFACE_OUTPUT) {
        // revpi_set_ao_value(&_pin, input);
    }
}

int RevPiAio::get()
{
    if (_type == Type::RR_HW_INTERFACE_INPUT) {
        return (revpi_get_ai_value(&_pin));
    } else {
        ROS_ERROR("Cannot read value of output type Aio %s.", _pin_str.c_str());
        return true;
    }
}

} // namespace rapyuta