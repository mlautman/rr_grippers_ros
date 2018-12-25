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
            std::cout << "**********" << _pin_str << std::endl;
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

// todo use array for status in pump class
// constexpr int numOfPumpStatus = 2;
class Pump {
public:
    Pump(const std::string& trigger_pin, const std::string& status_pin0
    , const std::string& status_pin1)
        : _trigger(trigger_pin, Gpio::Type::OUTPUT)
        , _status0(status_pin0, Gpio::Type::INPUT)
        , _status1(status_pin1, Gpio::Type::INPUT) {
    }

    bool init(BoardConfig& config) {
        return _trigger.init(config) && _status0.init(config) && _status1.init(config);
    }

    void enable() {
        _trigger.enable();
    }

    void disable() {
        _trigger.disable();
    }

    bool is_attached() {
        return (!_status0.value() && !_status1.value());
    }

    bool value(unsigned int num) {
        if(num == 0){
            _status0.value();
        }
        else if(num==1){
            _status1.value();
        }
    }


private:
    Gpio _trigger;
    Gpio _status0;
    Gpio _status1;
};

} // namespace rapyuta