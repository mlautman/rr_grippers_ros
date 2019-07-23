#pragma once

#include <revpi.h>
#include "rr_hw_interface/hw_interface.hpp"

namespace rapyuta
{


/*
    libsoc board conig class
*/
class RevPiAioBoardConfig
{
public:
    RevPiAioBoardConfig(){};
    ~RevPiAioBoardConfig(){};
};

/*
    aio interface class with RevPiAio
*/
class RevPiAio: public HwInterface<int, RevPiAioBoardConfig>
{
public:
    RevPiAio(const std::string& pin_str, const Type& type);
    ~RevPiAio();
    bool init(RevPiAioBoardConfig& config);
    void set(int input);
    int get(); 

private:
    revpi_peripheral _pin;
};

} // namespace rapyuta