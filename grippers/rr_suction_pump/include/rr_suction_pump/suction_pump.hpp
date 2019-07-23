#pragma once

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <rr_hw_interface/gpio/libsoc_gpio.hpp>
#include <rr_hw_interface/hw_interface.hpp>

namespace rapyuta
{

/*
    Pump interface which have trigger and status gpio class
*/
template <class HI, class Config>
class Pump
{
public:
    typedef std::unique_ptr<HI> HI_ptr;
    Pump(const std::vector<std::string> triggerPins, const int numOfTrigger, const std::vector<std::string> statusPins, const int numOfStatus,
            const bool outputNormallyOn, const bool inputNormallyOn)
            : _outputNormallyOn(outputNormallyOn)
            , _inputNormallyOn(inputNormallyOn)
    {
        for (int i = 0; i < numOfTrigger; i++) {
            _trigger.push_back(HI_ptr(new HI(triggerPins[i], HI::Type::OUTPUT)));
        }
        for (int i = 0; i < numOfStatus; i++) {
            _status.push_back(HI_ptr(new HI(statusPins[i], HI::Type::INPUT)));
        }
    };

    bool init(Config& config)
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
        if (_outputNormallyOn) {
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
        if (_outputNormallyOn) {
            _trigger[num]->set(!input);
        } else {
            _trigger[num]->set(input);
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
        bool val = _status[num]->get();
        if (_inputNormallyOn) {
            val = !val;
        }
        return val;
    };

private:
    std::vector<HI_ptr> _trigger;
    std::vector<HI_ptr> _status;
    bool _outputNormallyOn;
    bool _inputNormallyOn;
};

} // namespace rapyuta