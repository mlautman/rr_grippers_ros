#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

#include "suction_pump/suction_pump.hpp"

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

Gpio::Gpio(const std::string& pin_str, const Type& type)
        : _pin(NULL)
        , _pin_str(pin_str)
        , _type(type)
{
}

Gpio::~Gpio()
{
    if (_pin) {
        if (libsoc_gpio_free(_pin) == EXIT_FAILURE) {
            ROS_ERROR("Could not free gpio pin %s.", _pin_str.c_str());
        }
    }
}

bool Gpio::init(BoardConfig& config)
{
    _pin = libsoc_gpio_request(libsoc_board_gpio_id(config.get(), _pin_str.c_str()), LS_GPIO_SHARED);
    if (_pin == NULL) {
        ROS_ERROR("Gpio request for pin %s failed", _pin_str.c_str());
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

void Gpio::enable()
{
    if (_type == Type::OUTPUT) {
        libsoc_gpio_set_level(_pin, HIGH);
    }
}

void Gpio::disable()
{
    if (_type == Type::OUTPUT) {
        libsoc_gpio_set_level(_pin, LOW);
    }
}

bool Gpio::value()
{
    if (_type == Type::INPUT) {
        return libsoc_gpio_get_level(_pin);
    } else {
        ROS_ERROR("Cannot read value of output type gpio %s.", _pin_str.c_str());
        return true;
    }
}

Pump::Pump(const std::vector<std::string> triggerPins, const int numOfTrigger, const std::vector<std::string> statusPins, const int numOfStatus,
        const bool normallyOn)
        : _normallyOn(normallyOn)
{
    for (int i = 0; i < numOfTrigger; i++) {
        _trigger.push_back(Gpio_ptr(new Gpio(triggerPins[i], Gpio::Type::OUTPUT)));
    }
    for (int i = 0; i < numOfStatus; i++) {
        _status.push_back(Gpio_ptr(new Gpio(statusPins[i], Gpio::Type::INPUT)));
    }
}

bool Pump::init(BoardConfig& config)
{
    for (Gpio_ptr& trigger : _trigger) {
        if (!trigger->init(config)) {
            return false;
        }
    }
    for (Gpio_ptr& status : _status) {
        if (!status->init(config)) {
            return false;
        }
    }
    return true;
}

void Pump::enable()
{
    if (_normallyOn) {
        for (Gpio_ptr& trigger : _trigger) {
            trigger->disable();
        }
    } else {
        for (Gpio_ptr& trigger : _trigger) {
            trigger->enable();
        }
    }
}

void Pump::enable(unsigned int num)
{
    if (_normallyOn) {
        _trigger[num]->disable();
    } else {
        _trigger[num]->enable();
    }
}

void Pump::disable()
{
    if (_normallyOn) {
        for (Gpio_ptr& trigger : _trigger) {
            trigger->enable();
        }
    } else {
        for (Gpio_ptr& trigger : _trigger) {
            trigger->disable();
        }
    }
}

void Pump::disable(unsigned int num)
{
    if (_normallyOn) {
        _trigger[num]->enable();
    } else {
        _trigger[num]->disable();
    }
}

bool Pump::isAttached()
{
    for (Gpio_ptr& status : _status) {
        if (!status->value()) {
            return false;
        }
    }
    return true;
}

bool Pump::value(unsigned int num)
{
    return _status[num]->value();
}

} // namespace rapyuta