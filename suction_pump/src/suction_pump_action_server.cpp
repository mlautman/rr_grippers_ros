/*
Todo
- handle multiple pumps
- change number of status and pums to

Last Update 20181222, yu.okamoto@rapyuta-robotics.com
*/

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <suction_pump/SuctionPumpAction.h>
#include <suction_pump/suction_pump.hpp>
#include "suction_pump/suction_pump_action_server.hpp"

namespace rapyuta
{

constexpr int NUM_OF_PUMP_STATUS = 2;
constexpr char PUMP_TRIGGER_GPIO[] = "DIO1_PIN_11";
constexpr char PUMP_STATUS_GPIO[][20] = {"DIO1_PIN_1", "DIO1_PIN_2"};

class SuctionPumpActionServer
{
public:
    SuctionPumpActionServer(ros::NodeHandle& nh, const std::string& action_name)
            : _pump(PUMP_TRIGGER_GPIO, PUMP_STATUS_GPIO, NUM_OF_PUMP_STATUS, true)
            , _server(nh, action_name, boost::bind(&SuctionPumpActionServer::action_cb, this, _1), false)
            , _action_name(action_name)
    {
    }

    bool init()
    {
        if (_pump.init(_config)) {
            _server.start();
            _pump.disable(); // set off
            ROS_INFO("%s: Started", _action_name.c_str());
            return true;
        }
        return false;
    }

    void action_cb(const suction_pump::SuctionPumpGoalConstPtr& goal)
    {
        suction_pump::SuctionPumpFeedback feedback;
        feedback.data = NothingAttached;
        ros::Time start_time = ros::Time::now();
        if (goal->engage) {
            _pump.enable();
        } else {
            _pump.disable();
        }

        ros::Rate loop_rate(10);
        while ((ros::Time::now() - start_time) < ros::Duration(goal->timeout)) {
            if (_server.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", _action_name.c_str());
                _server.setPreempted();
                break;
            }
            if (goal->engage) {
                bool output[2] = {_pump.value(0), _pump.value(1)};
                ROS_INFO("Suction pomp status out1:%d, out2:%d", output[0], output[1]);

                if (!output[0]) {
                    feedback.data = NothingAttached;
                } else {
                    if (!output[1]) {
                        feedback.data = HalfAttached;
                    } else {
                        feedback.data = FullAttached;
                    }
                    _server.publishFeedback(feedback);
                    break;
                }
            } else {
                break;
            }
            _server.publishFeedback(feedback);
            loop_rate.sleep();
        }
        suction_pump::SuctionPumpResult result;
        result.data = feedback.data;
        if (feedback.data > 0 || !goal->engage) {
            _server.setSucceeded(result);
        } else {
            if (goal->engage) {
                _pump.disable();
            }
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "suction_pump_action_server");
    ros::NodeHandle nh;
    rapyuta::SuctionPumpActionServer spas(nh, "suction_pump_action_server");
    if (!spas.init()) {
        return 1;
    }
    ros::spin();
    return 0;
}
