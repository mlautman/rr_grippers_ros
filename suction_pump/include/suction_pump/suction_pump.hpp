#pragma once

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

#include <hw_interface/hw_interface.hpp>
#include <hw_interface/gpio.hpp>

namespace rapyuta
{

/*
    Pump interface which have trigger and status gpio class
*/
template <class HI>
class Pump
{
public:
    typedef std::unique_ptr<HI> HI_ptr;
    Pump(const std::vector<std::string> triggerPins, const int numOfTrigger, const std::vector<std::string> statusPins, const int numOfStatus,
            const bool normallyOn)
            : _normallyOn(normallyOn)
    {
        for (int i = 0; i < numOfTrigger; i++) {
            _trigger.push_back(HI_ptr(new HI(triggerPins[i], HI::Type::OUTPUT)));
        }
        for (int i = 0; i < numOfStatus; i++) {
            _status.push_back(HI_ptr(new HI(statusPins[i], HI::Type::INPUT)));
        }
    };

    bool init(BoardConfig& config)
    {
        for (HI_ptr& trigger : _trigger) {
            if (!trigger->init(config)) {
                return false;
            }
        }
        for (HI_ptr& status : _status) {
            if (!status->init(config)) {
                return false;
            }
        }
        return true;
    };

    void set(bool input)
    {
        if (_normallyOn) {
            for (HI_ptr& trigger : _trigger) {
                trigger->set(!input);
            }
        } else {
            for (HI_ptr& trigger : _trigger) {
                trigger->set(input);
            }
        }
    };

    void set(bool input, unsigned int num)
    {
        if (_normallyOn) {
            _trigger[num]->set(false);
        } else {
            _trigger[num]->set(true);
        }
    };

    bool isAttached()
    {
        for (HI_ptr& status : _status) {
            if (!status->get()) {
                return false;
            }
        }
        return true;
    };

    bool get(unsigned int num)
    {
        return _status[num]->get();
    };

private:
    std::vector<HI_ptr> _trigger;
    std::vector<HI_ptr> _status;
    bool _normallyOn;
};

} // namespace rapyuta