#include <ros/ros.h>
#include <std_msgs/Float32.h>

//todo change to librevpi
#include <rr_hw_interface/gpio/libsoc_gpio.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lr_tb5000c");
  ros::NodeHandle n;
  ros::Publisher sensor_pub = n.advertise<std_msgs::Float32>("lr_tb5000c", 10);

  //todo change to librevpi
  rapyuta::BoardConfig config;
  rapyuta::Gpio gpio("temp", rapyuta::Gpio::Type::INPUT);
  gpio.init(config);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    std_msgs::Float32 msg;
    msg.data = gpio.get(); 
    sensor_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}