#pragma once

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

namespace rapyuta
{

constexpr int GPIO_NAME_LENGTH=20;

class BoardConfig
{
public:
    BoardConfig();
    ~BoardConfig();
    board_config* get();

private:
    board_config* _bc;
};

class Gpio
{
public:
    enum class Type
    {
        INPUT,
        OUTPUT
    };
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

class Pump
{
public:
    typedef std::unique_ptr<Gpio> Gpio_ptr;
    Pump(const char triggerPins[][GPIO_NAME_LENGTH], const int numOfTrigger, 
         const char statusPins[][GPIO_NAME_LENGTH], const int numOfStatus, const bool normallyOn);

    bool init(BoardConfig& config);
    void enable();
    void enable(unsigned int num);
    void disable();
    void disable(unsigned int num);
    bool isAttached();
    bool value(unsigned int num);

private:
    std::vector<Gpio_ptr> _trigger;
    std::vector<Gpio_ptr> _status;
    bool _normallyOn;
};

} // namespace rapyuta