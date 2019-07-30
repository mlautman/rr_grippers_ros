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
    McpGpioBoardConfig(){};
    ~McpGpioBoardConfig(){};
    void set_mcp_address(int mcp_address)
    {
        _mcp_address = mcp_address;
    };
    void set_i2c_bus_instance(int i2c_bus_instance)
    {
        _i2c_bus_instance = i2c_bus_instance;
    };
    int return_mcp_address()
    {
        return _mcp_address;
    };
    int return_i2c_bus_instance()
    {
        return _i2c_bus_instance;
    };

private:
    int _mcp_address;
    int _i2c_bus_instance;
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
    //void set(int pin,bool input);
    //bool get(int pin);
    bool get();
    //void pinmode_input(int pin,uint8_t direction);
    void pinmode(uint8_t direction);

private:
    uint8_t _pin;
    i2c* _i2c_mcp23017;
};

} // namespace rapyuta