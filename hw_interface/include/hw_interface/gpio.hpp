#pragma once

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

#include "hw_interface.hpp"

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
class Gpio: public HwInterface<bool, BoardConfig>
{
public:
    Gpio(const std::string& pin_str, const Type& type);
    ~Gpio();
    bool init(BoardConfig& config);
    void set(bool input);
    bool get(); 

private:
    gpio* _pin;
};

} // namespace rapyuta