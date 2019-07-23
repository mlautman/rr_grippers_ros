#pragma once

#include "rr_hw_interface/hw_interface.hpp"
#include <revpi.h>

namespace rapyuta
{

/*
    libsoc board conig class
*/
class RevPiGpioBoardConfig
{
public:
    RevPiGpioBoardConfig(){};
    ~RevPiGpioBoardConfig(){};
};

/*
    gpio interface class with RevPiGpio
*/
class RevPiGpio : public HwInterface<bool, RevPiGpioBoardConfig>
{
public:
    RevPiGpio(const std::string& pin_str, const Type& type);
    ~RevPiGpio();
    bool init(RevPiGpioBoardConfig& config);
    void set(bool input);
    bool get();

private:
    revpi_peripheral _pin;
};

} // namespace rapyuta