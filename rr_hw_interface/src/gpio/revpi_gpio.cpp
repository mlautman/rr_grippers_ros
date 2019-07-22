#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include "rr_hw_interface/gpio/revpi_gpio.hpp"

namespace rapyuta
{

RevPiGpio::RevPiGpio(const std::string& pin_str, const Type& type)
        : HwInterface(pin_str, type){
}

RevPiGpio::~RevPiGpio()
{
}

bool RevPiGpio::init(RevPiGpioBoardConfig& config)
{
    sprintf(_pin.pin_name, "%s", _pin_str.c_str());
    int ret = revpi_init(&_pin);
    if (ret<0) {
        ROS_ERROR("RevPiGpio request for pin %s failed", _pin_str.c_str());
        return false;
    }

    return true;
}

void RevPiGpio::set(bool input)
{
    if (_type == Type::OUTPUT) {
        revpi_set_do_level(&_pin, input);
    }
}


bool RevPiGpio::get()
{
    if (_type == Type::INPUT) {
        return (revpi_get_di_level(&_pin));
    } else {
        ROS_ERROR("Cannot read value of output type gpio %s.", _pin_str.c_str());
        return true;
    }
}

} // namespace rapyuta