#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <rr_hw_interface/hw_interface.hpp>

namespace rapyuta
{

float linear_trans(float a, float b, int input)
{
    return a * input + b;
}

float linear_trans(float a, float b, int input, float max, float min)
{
    float output = linear_trans(a, b, input);
    if (output > max) {
        output = max;
    } else if (output < min) {
        output = min;
    }
    return output;
}

/*
    sensor interface which have trigger and status gpio class
*/
template <class HI, class Config>
class AnalogSensor
{
public:
    AnalogSensor(float incline, float offset, float max, float min, std::string name, ros::NodeHandle n, std::string topic_name)
            : _incline(incline)
            , _offset(offset)
            , _max(max)
            , _min(min)
            , _name(name)
            , _analog(_name, HI::Type::RR_HW_INTERFACE_INPUT)
    {
        _pub = n.advertise<std_msgs::Float32>(topic_name, 10);
    };

    bool init(Config& config) { return _analog.init(config); };

    float get() { return linear_trans(_incline, _offset, _analog.get(), _max, _min); };

    float pub()
    {
        std_msgs::Float32 msg;
        msg.data = get();
        _pub.publish(msg);
    }

private:
    float _incline;
    float _offset;
    float _max;
    float _min;
    std::string _name;
    HI _analog;
    ros::Publisher _pub;
};

} // namespace rapyuta