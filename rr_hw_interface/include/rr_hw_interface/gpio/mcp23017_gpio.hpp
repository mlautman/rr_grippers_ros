#pragma once

#include "rr_hw_interface/hw_interface.hpp"
#include <mcp23017.h>

namespace rapyuta
{

/*
    libsoc board conig class
*/
class McpGpioBoardConfig
{
public:
    McpGpioBoardConfig(uint8_t i2c_bus_instance, uint8_t mcp_address);
    ~McpGpioBoardConfig();
    i2c* get_i2c();

private:
    i2c* _i2c_mcp23017;
    bool _status; //true -> ready, false -> error
};

/*
    gpio interface class with MCP23017
*/
class McpGpio : public HwInterface<bool, McpGpioBoardConfig>
{
public:
    McpGpio(const std::string& pin_str, const Type& type);
    ~McpGpio();
    bool init(McpGpioBoardConfig& config);
    void set(bool input);
    bool get();

private:
    uint8_t _pin;
    i2c* _i2c_mcp23017;
};

} // namespace rapyuta