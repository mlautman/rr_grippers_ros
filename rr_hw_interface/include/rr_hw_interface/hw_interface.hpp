#pragma once

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

namespace rapyuta
{

/*
   HW interface class with libsoc
*/
template <class T, class Config>
class HwInterface
{
public:
    enum class Type
    {
        INPUT,
        OUTPUT,
    };
    HwInterface(){};
    HwInterface(const std::string& pin_str, const Type& type)
            : _pin_str(pin_str)
            , _type(type){};
    virtual ~HwInterface(){};
    virtual bool init(Config& config) = 0;
    virtual void set(T input) = 0; // set pin high
    virtual T get() = 0;           // return pin level

protected:
    std::string _pin_str;
    Type _type;
};

} // namespace rapyuta