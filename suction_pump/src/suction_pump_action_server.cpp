/*
Todo: Create parent class which handle Suctionpump and children class which handle single/double pu
Last Update 20181222, yu.okamoto@rapyuta-robotics.com
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <suction_pump/suction_pump.hpp>
#include <suction_pump/SuctionPumpAction.h>

#define PUMP_TRIGGER_GPIO "DIO1_PIN_11"
#define PUMP_STATUS_GPIO0 "DIO1_PIN_1"
#define PUMP_STATUS_GPIO1 "DIO1_PIN_2"

namespace rapyuta {

enum PumpFeeedback{
    NothingSucked = 0,
    HalfSucked = 1,
    FullSucked = 2
};

class SuctionPumpActionServer {
public:
    SuctionPumpActionServer(ros::NodeHandle& nh, const std::string& action_name)
            : _pump(PUMP_TRIGGER_GPIO, PUMP_STATUS_GPIO0, PUMP_STATUS_GPIO1)
            , _server(nh, action_name, boost::bind(&SuctionPumpActionServer::action_cb, this, _1), false)
            , _action_name(action_name) {
    }

    bool init() {
        if (_pump.init(_config)){
            _server.start();
            _pump.enable(); // set off
            ROS_INFO("%s: Started", _action_name.c_str());
            return true;
        }
        return false;
    }

    void action_cb(const suction_pump::SuctionPumpGoalConstPtr& goal) {
        suction_pump::SuctionPumpFeedback feedback;
        feedback.data = NothingSucked; 
        ros::Time start_time = ros::Time::now();
        if (goal->engage) {
            _pump.disable(); //FXP-SW is normally off. Disable -> sucking
        } else {
            _pump.enable();
        }

        ros::Rate loop_rate(10);
        while((ros::Time::now() - start_time) < ros::Duration(goal->timeout)) {
            if (_server.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", _action_name.c_str());
                _server.setPreempted();
                break;
            }
            if (goal->engage){
                bool output[2] = {_pump.value(0), _pump.value(1)}; 
                ROS_INFO("Suction pomp status out1:%d, out2:%d", output[0], output[1]);
                
                if(!output[0]){
                    feedback.data = NothingSucked;
                }
                else{
                    if(!output[1]){
                        feedback.data= HalfSucked;
                    }
                    else{
                        feedback.data= FullSucked;
                    }
                    break;
                }
            }
            else{
                break;
            }
            _server.publishFeedback(feedback);
            loop_rate.sleep();
        }
        suction_pump::SuctionPumpResult result;
        result.data = true;
        if (feedback.data || !goal->engage) {
            _server.setSucceeded(result);
        } else {
            if (goal->engage) {
                _pump.enable();
            }
            result.data = false;
            ROS_ERROR("%s: Aborted", _action_name.c_str());
            _server.setAborted(result);
        }
    }

private:
    Pump _pump;
    BoardConfig _config;
    actionlib::SimpleActionServer<suction_pump::SuctionPumpAction> _server;
    std::string _action_name;
};

} // namespace rapyuta

int main(int argc, char** argv) {
    ros::init(argc, argv, "suction_pump_action_server");
    ros::NodeHandle nh;
    rapyuta::SuctionPumpActionServer spas(nh, "suction_pump_action_server");
    if (!spas.init()) {
        return 1;
    }
    ros::spin();
    return 0;
}
