#pragma once

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <rr_hw_interface/hw_interface.hpp>

namespace rapyuta
{

/*
    Pump interface which have trigger and status gpio class
*/
template <class HI_Trigger, class HI_Status, class Config_Trigger, class Config_Status, class TI, class TO>
class Pump
{
public:
    typedef std::unique_ptr<HI_Trigger> HI_Trigger_ptr;
    typedef std::unique_ptr<HI_Status> HI_Status_ptr;
    Pump(const std::vector<std::string> triggerPins, const int numOfTrigger, const std::vector<std::string> statusPins, const int numOfStatus,
            const bool outputNormallyOn, const bool inputNormallyOn)
            : _outputNormallyOn(outputNormallyOn)
            , _inputNormallyOn(inputNormallyOn)
    {
        for (int i = 0; i < numOfTrigger; i++) {
            _trigger.push_back(HI_Trigger_ptr(new HI_Trigger(triggerPins[i], HI_Trigger::Type::RR_HW_INTERFACE_OUTPUT)));
        }
        for (int i = 0; i < numOfStatus; i++) {
            _status.push_back(HI_Status_ptr(new HI_Status(statusPins[i], HI_Status::Type::RR_HW_INTERFACE_INPUT)));
        }
    };

    bool init(Config_Trigger& config_trigger, Config_Status& config_status)
    {
        for (HI_Trigger_ptr& trigger : _trigger) {
            if (!trigger->init(config_trigger)) {
                return false;
            }
        }
        for (HI_Status_ptr& status : _status) {
            if (!status->init(config_status)) {
                return false;
            }
        }
        return true;
    };

    void set(TO input)
    {
        if (_outputNormallyOn) {
            for (HI_Trigger_ptr& trigger : _trigger) {
                trigger->set(!input);
            }
        } else {
            for (HI_Trigger_ptr& trigger : _trigger) {
                trigger->set(input);
            }
        }
    };

    void set(TO input, unsigned int num)
    {
        if (_outputNormallyOn) {
            _trigger[num]->set(!input);
        } else {
            _trigger[num]->set(input);
        }
    };

    bool isAttached()
    {
        for (HI_Status_ptr& status : _status) {
            if (!status->get()) {
                return false;
            }
        }
        return true;
    };

    TI get(unsigned int num)
    {
        TI val = _status[num]->get();
        if (_inputNormallyOn) {
            val = !val;
        }
        return val;
    };

private:
    std::vector<HI_Trigger_ptr> _trigger;
    std::vector<HI_Status_ptr> _status;
    bool _outputNormallyOn;
    bool _inputNormallyOn;
};

} // namespace rapyuta