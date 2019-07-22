#include <ros/ros.h>
#include <std_msgs/Float32.h>

//todo change to analog
// #include <rr_hw_interface/gpio/libsoc_gpio.hpp>

float linear_trans(float a, float b, int input){
  return a*input+b;
}

float linear_trans(float a, float b, int input, float max, float min){
  float output = linear_trans(a,b,input);
  if(output>max){
    output = max;
  }else if(output<min){
    output = min;
  }
  return output;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ao_sensor");
  ros::NodeHandle n;
  ros::Publisher sensor_pub = n.advertise<std_msgs::Float32>("analog_sensor", 10);

  std::string pin_name;
  float incline, offset, min, max;
  n.getParam("pin", pin_name);
  n.getParam("incline", incline);
  n.getParam("offset", offset);
  n.getParam("max", pin_name);
  n.getParam("min", pin_name);
  // todo change to analog
  // rapyuta::LibsocBoardConfig config;
  // rapyuta::LibsocGpio analog(pin_name, rapyuta::LibsocGpio::Type::INPUT);
  // analog.init(config);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    std_msgs::Float32 msg;
    //todo proper conversion from analog to distance[m]
    float data; 
    msg.data = linear_trans(incline, offset, max, min, data);
    sensor_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}