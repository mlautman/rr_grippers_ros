#pragma once

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

#include <hw_interface/gpio.hpp>

namespace rapyuta
{

/*
    Pump interface which have trigger and status gpio class
*/
class Pump
{
public:
    typedef std::unique_ptr<Gpio> Gpio_ptr;
    Pump(const std::vector<std::string> triggerPins, const int numOfTrigger, const std::vector<std::string> statusPins, const int numOfStatus,
            const bool normallyOn = false);

    bool init(BoardConfig& config);
    void enable();                  // set all trigger pin high
    void enable(unsigned int num);  // set pin at num high
    void disable();                 // set all trigger pin low
    void disable(unsigned int num); // set pin at num low
    bool isAttached();              // return true if all status pin are high
    bool value(unsigned int num);   // return pin at num level

private:
    std::vector<Gpio_ptr> _trigger;
    std::vector<Gpio_ptr> _status;
    bool _normallyOn;
};

} // namespace rapyuta