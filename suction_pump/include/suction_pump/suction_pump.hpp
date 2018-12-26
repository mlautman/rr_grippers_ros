#pragma once

#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include <ros/ros.h>

#include <libsoc_gpio.h>
#include <libsoc_board.h>

namespace rapyuta {

class BoardConfig {
public:
    BoardConfig() {
        _bc = libsoc_board_init();
    }

    ~BoardConfig() {
        if (_bc) {
            free(_bc);
            _bc = NULL;
        }
    }

    board_config* get() {
        return _bc;
    }

private:
    board_config* _bc;
};

class Gpio {
public:
    enum class Type { INPUT, OUTPUT};

    explicit Gpio(const std::string& pin_str, const Type& type) 
        : _pin(NULL)
        , _pin_str(pin_str)
        , _type(type) {
    }

    ~Gpio() {
        if (_pin) {
            if (libsoc_gpio_free(_pin) == EXIT_FAILURE) {
                ROS_ERROR("Could not free gpio pin %s.", _pin_str.c_str());
            }
        }
    }

    bool init(BoardConfig& config) {
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

    void enable() {
        if (_type == Type::OUTPUT) {
            libsoc_gpio_set_level(_pin, HIGH);
        }
    }

    void disable() {
        if (_type == Type::OUTPUT) {
            libsoc_gpio_set_level(_pin, LOW);
        }
    }

    bool value() {
        if (_type == Type::INPUT) {
            return libsoc_gpio_get_level(_pin);
        } else {
            ROS_ERROR("Cannot read value of output type gpio %s.", _pin_str.c_str());
            return true;
        }
    }

private:
    gpio* _pin;
    std::string _pin_str;
    Type _type;
};

class Pump {
typedef std::unique_ptr<Gpio> Gpio_ptr;
public:
    Pump(const std::string& trigger_pin, const char status_pins[][20], const int numOfStatus,
        const bool normallyOn=false)
        : _trigger(trigger_pin, Gpio::Type::OUTPUT), _normallyOn(normallyOn)
        {   
            for (int i=0; i< numOfStatus; i++){
                _status.push_back(Gpio_ptr(new Gpio(status_pins[i], Gpio::Type::INPUT)));
            }
    }

    bool init(BoardConfig& config) {
        for(Gpio_ptr& status: _status){
            if(!status->init(config)){
                return false;
            }
        }        
        return _trigger.init(config);
    }

    void enable() {
        if(_normallyOn){
            _trigger.disable();
        }
        else{
            _trigger.enable();
        }
    }

    void disable() {
        if(_normallyOn){
            _trigger.enable();
        }
        else{
            _trigger.disable();
        }
    }

    bool is_attached() {
        for(Gpio_ptr& status: _status){
            if(!status->value()){
                return false;
            }
        }
        return true;
    }

    bool value(unsigned int num) {
        return _status[num]->value();
    }


private:
    Gpio _trigger;
    std::vector<Gpio_ptr>  _status;
    bool _normallyOn;
};

} // namespace rapyuta