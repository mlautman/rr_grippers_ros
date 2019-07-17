#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

#include "suction_pump/suction_pump.hpp"

namespace rapyuta
{

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
            trigger->set(false);
        }
    } else {
        for (Gpio_ptr& trigger : _trigger) {
            trigger->set(true);
        }
    }
}

void Pump::enable(unsigned int num)
{
    if (_normallyOn) {
        _trigger[num]->set(false);
    } else {
        _trigger[num]->set(true);
    }
}

void Pump::disable()
{
    if (_normallyOn) {
        for (Gpio_ptr& trigger : _trigger) {
            trigger->set(true);
        }
    } else {
        for (Gpio_ptr& trigger : _trigger) {
            trigger->set(false);
        }
    }
}

void Pump::disable(unsigned int num)
{
    if (_normallyOn) {
        _trigger[num]->set(true);
    } else {
        _trigger[num]->set(false);
    }
}

bool Pump::isAttached()
{
    for (Gpio_ptr& status : _status) {
        if (!status->get()) {
            return false;
        }
    }
    return true;
}

bool Pump::value(unsigned int num)
{
    return _status[num]->get();
}

} // namespace rapyuta