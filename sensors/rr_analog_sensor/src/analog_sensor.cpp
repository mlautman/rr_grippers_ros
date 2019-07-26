#include <ros/ros.h>
#include <std_msgs/Float32.h>

// todo change to analog
#include <rr_hw_interface/aio/revpi_aio.hpp>

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "analog_sensor");
    ros::NodeHandle n;
    ros::Publisher sensor_pub = n.advertise<std_msgs::Float32>("analog_sensor", 10);

    std::string pin_name;
    float incline, offset, min, max;
    n.getParam("pin", pin_name);
    n.getParam("incline", incline);
    n.getParam("offset", offset);
    n.getParam("max", max);
    n.getParam("min", min);
    rapyuta::RevPiAioBoardConfig config;
    rapyuta::RevPiAio analog(pin_name, rapyuta::RevPiAio::Type::RR_HW_INTERFACE_INPUT);
    analog.init(config);

    ros::Rate loop_rate(50);
    std_msgs::Float32 msg;
    while (ros::ok()) {
        msg.data = linear_trans(incline, offset, analog.get(), max, min);
        sensor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}