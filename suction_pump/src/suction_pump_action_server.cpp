#/*
 todo add SuctionPumpActionServer class and two children class which handle single pump and double pump
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <suction_pump/suction_pump.hpp>
#include <suction_pump/SuctionPumpAction.h>

#define PUMP_A_TRIGGER_GPIO "DIO1_PIN_11"
#define PUMP_A_STATUS_GPIO "DIO1_PIN_1"
// #define PUMP_B_TRIGGER_GPIO "DIO1_PIN_12"
// #define PUMP_B_STATUS_GPIO "DIO1_PIN_2"

namespace rapyuta {

class SuctionPumpActionServer {
public:
    SuctionPumpActionServer(ros::NodeHandle& nh, const std::string& action_name)
            : _pump_a(PUMP_A_TRIGGER_GPIO, PUMP_A_STATUS_GPIO)
            // , _pump_b(PUMP_B_TRIGGER_GPIO, PUMP_B_STATUS_GPIO)
            , _server(nh, action_name, boost::bind(&SuctionPumpActionServer::action_cb, this, _1), false)
            , _action_name(action_name) {
    }

    bool init() {
        if (_pump_a.init(_config)){// && _pump_b.init(_config)) {
            _server.start();
            ROS_INFO("%s: Started", _action_name.c_str());
            return true;
        }
        return false;
    }

    void action_cb(const suction_pump::SuctionPumpGoalConstPtr& goal) {
        suction_pump::SuctionPumpFeedback feedback;
        feedback.data = true; // @todo FIXME: This should be false
        ros::Time start_time = ros::Time::now();
        if (goal->engage) {
            _pump_a.enable();
            // _pump_b.enable();
        } else {
            _pump_a.disable();
            // _pump_b.disable();
        }
        while((ros::Time::now() - start_time) < ros::Duration(goal->timeout)) {
            if (_server.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", _action_name.c_str());
                _server.setPreempted();
                break;
            }
            if (goal->engage && _pump_a.is_attached()){// && _pump_b.is_attached()) {
                ROS_INFO("%s: Suction pump attached", _action_name.c_str());
                feedback.data = true;
                break;
            }
            if (!goal->engage && !_pump_a.is_attached()){ //&& !_pump_b.is_attached()) {
                ROS_INFO("%s: Suction pump detached", _action_name.c_str());
                feedback.data = true;
                break;
            }
            _server.publishFeedback(feedback);
        }
        suction_pump::SuctionPumpResult result;
        result.data = true;
        if (feedback.data) {
            _server.setSucceeded(result);
        } else {
            if (goal->engage) {
                _pump_a.disable();
                // _pump_b.disable();
            }
            ROS_ERROR("%s: Aborted", _action_name.c_str());
            _server.setAborted(result);
        }
    }

private:
    Pump _pump_a;
    // Pump _pump_b;
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