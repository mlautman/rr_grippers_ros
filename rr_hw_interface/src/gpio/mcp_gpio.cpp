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

McpGpio::McpGpio(const std::string& pin_str, const Type& type)
        : HwInterface(pin_str, type)
{
}

McpGpio::~McpGpio()
{
    libsoc_i2c_free(_i2c_mcp23017);
}

bool McpGpio::init(McpGpioBoardConfig& config)
{
    _i2c_mcp23017 = mcp23xx_init(config.return_i2c_bus_instance(), config.return_mcp_address());
    if (_i2c_mcp23017 == NULL) {
        ROS_ERROR("MCP23017 init failed ");
        return false;
    }
    ROS_INFO("MCP23017 init passed ");
    return true;
}

void McpGpio::set(bool input)
{
    _pin =std::stoi( _pin_str );
    ROS_INFO("'%d' is the converted pin number in mcpgpio set",_pin);
    if (_i2c_mcp23017 != NULL) {
        if (_type == Type::RR_HW_INTERFACE_OUTPUT) {
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

void McpGpio::pinmode(uint8_t direction)
{
    _pin =std::stoi( _pin_str );
    mcp_pinMode(_i2c_mcp23017, _pin, (direction == INPUT));
}

bool McpGpio::get()
{
    _pin =std::stoi( _pin_str );
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