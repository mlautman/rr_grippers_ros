#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include <libsoc_board.h>
#include <libsoc_gpio.h>

#include "mcp23017.h"
#include <rr_hw_interface/gpio/mcp23017_gpio.hpp>
namespace rapyuta
{

McpGpioBoardConfig::McpGpioBoardConfig(uint8_t i2c_bus_instance, uint8_t mcp_address)
    :_i2c_mcp23017(NULL), _status(false)
{
    _status = mcp23xx_init(i2c_bus_instance, mcp_address, &_i2c_mcp23017);
};

McpGpioBoardConfig::~McpGpioBoardConfig()
{
    if (_i2c_mcp23017) {
        libsoc_i2c_free(_i2c_mcp23017);
    }
};

i2c* McpGpioBoardConfig::get_i2c()
{
    if(_status){
        return _i2c_mcp23017;
    }
    return NULL;
    
};

McpGpio::McpGpio(const std::string& pin_str, const Type& type)
        : _pin(std::stoi(pin_str))
        , HwInterface(pin_str, type)
{
}

McpGpio::~McpGpio()
{
}

bool McpGpio::init(McpGpioBoardConfig& config)
{
    _i2c_mcp23017 = config.get_i2c();
    if (_i2c_mcp23017 == NULL) {
        ROS_ERROR("MCP23017 init failed ");
        return false;
    }
    if (_type == Type::RR_HW_INTERFACE_INPUT) {
        mcp_pinMode(_i2c_mcp23017, _pin, OUTPUT); // see from mcp side
    } else {
        mcp_pinMode(_i2c_mcp23017, _pin, INPUT);
    }
    ROS_INFO("MCP23017 init passed ");

    return true;
}

void McpGpio::set(bool input)
{
    if (_i2c_mcp23017 != NULL) {
        if (_type == Type::RR_HW_INTERFACE_INPUT) {
            if (input) {
                mcp_digitalWrite(_i2c_mcp23017, _pin, HIGH);
            } else {
                mcp_digitalWrite(_i2c_mcp23017, _pin, LOW);
            }
        }
    } else {
        ROS_ERROR("MCP GPIO set failed");
    }
}

bool McpGpio::get()
{
    if (_i2c_mcp23017 != NULL) {
        if (_type == Type::RR_HW_INTERFACE_INPUT) {
            return mcp_digitalRead(_i2c_mcp23017, _pin);
        }
    } else {
        ROS_ERROR("Cannot read value of output type gpio %s.", _pin_str.c_str());
        return true;
    }
}

} // namespace rapyuta