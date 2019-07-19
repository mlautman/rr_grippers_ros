#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

#include "rr_libsoc_gpio/libsoc_gpio.hpp"

namespace rapyuta
{

BoardConfig::BoardConfig()
{
    _bc = libsoc_board_init();
}

BoardConfig::~BoardConfig()
{
    if (_bc) {
        free(_bc);
        _bc = NULL;
    }
}

board_config* BoardConfig::get()
{
    return _bc;
}

LibsocGpio::LibsocGpio(const std::string& pin_str, const Type& type)
        : _pin(NULL), HwInterface(pin_str, type){
}

LibsocGpio::~LibsocGpio()
{
    if (_pin) {
        if (libsoc_gpio_free(_pin) == EXIT_FAILURE) {
            ROS_ERROR("Could not free gpio pin %s.", _pin_str.c_str());
        }
    }
}

bool LibsocGpio::init(BoardConfig& config)
{
    _pin = libsoc_gpio_request(libsoc_board_gpio_id(config.get(), _pin_str.c_str()), LS_GPIO_SHARED);
    if (_pin == NULL) {
        ROS_ERROR("LibsocGpio request for pin %s failed", _pin_str.c_str());
        return false;
    }

    int ret;
    if (_type == Type::OUTPUT) {
        ret = libsoc_gpio_set_direction(_pin, OUTPUT);
    } else {
        ret = libsoc_gpio_set_direction(_pin, INPUT);
    }
    if (ret == EXIT_FAILURE) {
        ROS_ERROR("Failed to set gpio direction for pin %s.", _pin_str.c_str());
        return false;
    }
    return true;
}

void LibsocGpio::set(bool input)
{
    if (_type == Type::OUTPUT) {
        if(input){
            libsoc_gpio_set_level(_pin, HIGH);
        }else{
            libsoc_gpio_set_level(_pin, LOW);
        }
    }
}


bool LibsocGpio::get()
{
    if (_type == Type::INPUT) {
        return libsoc_gpio_get_level(_pin);
    } else {
        ROS_ERROR("Cannot read value of output type gpio %s.", _pin_str.c_str());
        return true;
    }
}

} // namespace rapyuta