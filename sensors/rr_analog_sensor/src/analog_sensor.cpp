#include <ros/ros.h>
#include <std_msgs/Float32.h>

//todo change to analog
#include <rr_hw_interface/gpio/libsoc_gpio.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ao_sensor");
  ros::NodeHandle n;
  ros::Publisher sensor_pub = n.advertise<std_msgs::Float32>("analog_sensor", 10);

  //todo change to analog
  std::string pin_name;
  n.getParam("pin", pin_name);
  rapyuta::LibsocBoardConfig config;
  rapyuta::LibsocGpio analog(pin_name, rapyuta::LibsocGpio::Type::INPUT);
  analog.init(config);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    std_msgs::Float32 msg;
    //todo proper conversion from analog to distance[m]
    msg.data = analog.get(); 
    sensor_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}