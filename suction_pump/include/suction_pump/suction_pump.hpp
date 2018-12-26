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
    BoardConfig();
    ~BoardConfig();
    board_config* get();

private:
    board_config* _bc;
};

class Gpio {
public:
    enum class Type { INPUT, OUTPUT};
    explicit Gpio(const std::string& pin_str, const Type& type); 
    ~Gpio();
    bool init(BoardConfig& config);
    void enable();
    void disable();
    bool value();

private:
    gpio* _pin;
    std::string _pin_str;
    Type _type;
};

class Pump {
public:
    typedef std::unique_ptr<Gpio> Gpio_ptr;
    Pump(const std::string& trigger_pin, const char status_pins[][20], const int numOfStatus,
        const bool normallyOn=false);
    bool init(BoardConfig& config);
    void enable();
    void disable();
    bool is_attached();
    bool value(unsigned int num);

private:
    Gpio _trigger;
    std::vector<Gpio_ptr>  _status;
    bool _normallyOn;
};

} // namespace rapyuta