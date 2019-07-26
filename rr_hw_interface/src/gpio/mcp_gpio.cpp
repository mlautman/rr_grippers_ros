#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include <ros/ros.h>

#include "rr_hw_interface/gpio/mcp23017_gpio.hpp"

namespace rapyuta
{

McpGpio::McpGpio(const std::string& pin_str, const Type& type)
        : HwInterface(pin_str, type)
{
}

McpGpio::~McpGpio()
{
}

bool McpGpio::init(McpGpioBoardConfig& config)
{
    _i2c_mcp23017 = mcp23xx_init(config.return_i2c_bus_instance(), config.return_mcp_address());
    if (_i2c_mcp23017 == NULL){
        ROS_ERROR("MCP23017 init failed ");
        return false;
    }
    return true;
}

void McpGpio::set(bool input) 
{
    if (_i2c_mcp23017 != NULL){
        if (_type == Type::RR_HW_INTERFACE_OUTPUT) {
            mcp_digitalWrite(_i2c_mcp23017, _pin, input);
        }
    }
    else{
        ROS_ERROR("MCP GPIO set failed");
    }
}

bool McpGpio::get()
{
    if (_i2c_mcp23017 != NULL){
        if (_type == Type::RR_HW_INTERFACE_INPUT) {
            mcp_digitalRead(_i2c_mcp23017, _pin);
        }
    }
    else{
        ROS_ERROR("Cannot read value of output type gpio %s.", _pin_str.c_str());
        return true;
    }
    return true;
}
} // namespace rapyuta