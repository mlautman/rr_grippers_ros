#pragma once

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

namespace rapyuta
{

constexpr int GPIO_NAME_LENGTH = 20;

/*
    libsoc board conig class
*/
class BoardConfig
{
public:
    BoardConfig();
    ~BoardConfig();
    board_config* get();

private:
    board_config* _bc;
};

/*
    gpio interface class with libsoc
*/
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
    void enable();  // set pin high
    void disable(); // set pin low
    bool value();   // return pin level

private:
    gpio* _pin;
    std::string _pin_str;
    Type _type;
};

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