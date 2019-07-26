#include <stdio.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <rr_analog_sensor/analog_sensor.hpp>
#include <rr_hw_interface/aio/revpi_aio.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "analog_sensor");
    ros::NodeHandle n("~");

    std::string pin_name, topic_name;
    int sensorNum, sensorFreq;
    float incline, offset, min, max;

    // default parameters
    float incline_default, offset_default, min_default, max_default;
    n.getParam("incline", incline_default);
    n.getParam("offset", offset_default);
    n.getParam("max", max_default);
    n.getParam("min", min_default);

    n.getParam("sensor_num", sensorNum);
    n.getParam("sensor_freq", sensorFreq);

    std::cout << incline_default << " " 
              << offset_default << " " 
              << max_default << " " 
              << min_default << " " 
              << sensorNum << " "
              << sensorFreq << " "
              << std::endl;


    std::vector<rapyuta::AnalogSensor<rapyuta::RevPiAio, rapyuta::RevPiAioBoardConfig>> pins;
    for (int i = 0; i < sensorNum; i++) {

        std::string pin_num = "pin" + std::to_string(i);
        n.getParam(pin_num + "/name", pin_name);
        n.getParam(pin_num + "/topic_name", topic_name);
        ros::param::param<float>("~" + pin_num + "/incline", incline, incline_default);
        ros::param::param<float>("~" + pin_num + "/offset", offset, offset_default);
        ros::param::param<float>("~" + pin_num + "/max", max, max_default);
        ros::param::param<float>("~" + pin_num + "/min", min, min_default);
        pins.push_back(rapyuta::AnalogSensor<rapyuta::RevPiAio, rapyuta::RevPiAioBoardConfig>(incline, offset, max, min, pin_name, n, topic_name));
        std::cout << pin_name << " " 
                  << topic_name << " " 
                  << incline << " " 
                  << max << " " 
                  << min << " " 
                  << std::endl;
        rapyuta::RevPiAioBoardConfig config;
        pins.back().init(config);
    }
    ros::Rate loop_rate(sensorFreq);
    std_msgs::Float32 msg;
    while (ros::ok()) {
        for (auto itr = pins.begin(); itr != pins.end(); ++itr) {
            itr->pub();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}