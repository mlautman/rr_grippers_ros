/*
Last Update 20181222, yu.okamoto@rapyuta-robotics.com
*/

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <suction_pump/SuctionPumpAction.h>
#include <suction_pump/suction_pump.hpp>

namespace rapyuta
{

class SuctionPumpActionServer
{
public:
    SuctionPumpActionServer(ros::NodeHandle& nh, const std::string& action_name)
            : _server(nh, action_name, boost::bind(&SuctionPumpActionServer::action_cb, this, _1), false)
            , _action_name(action_name)
    {
        int trigger_num=0;
        int status_num=0;
        bool normallyOn=false;
        std::vector<std::string> trigger_gpio;
        std::vector<std::string> status_gpio;
        std::string trigger, status;

        nh.getParam("trigger_num", trigger_num);
        nh.getParam("status_num", status_num);
        nh.getParam("normally_on", normallyOn);
        for(int i=0; i<trigger_num; i++){
            nh.getParam("trigger_gpio"+std::to_string(i), trigger);
            trigger_gpio.push_back(trigger);
        }
        for(int i=0; i<status_num; i++){
            nh.getParam("status_gpio"+std::to_string(i), status);
            status_gpio.push_back(status);
        }
        _pump = std::unique_ptr<Pump>(new Pump(trigger_gpio, trigger_num, status_gpio, status_num, normallyOn));
    }

    bool init()
    {
        if (_pump->init(_config)) {
            _server.start();
            _pump->disable(); // set off
            ROS_INFO("%s: Started", _action_name.c_str());
            return true;
        }
        return false;
    }

    void action_cb(const suction_pump::SuctionPumpGoalConstPtr& goal)
    {
        suction_pump::SuctionPumpFeedback feedback;
        feedback.status = goal->NOTHING;
        ros::Time start_time = ros::Time::now();
        if (goal->engage) {
            _pump->enable();
        } else {
            _pump->disable();
        }

        ros::Rate loop_rate(10);
        while ((ros::Time::now() - start_time) < ros::Duration(goal->timeout)) {
            if (_server.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", _action_name.c_str());
                _server.setPreempted();
                break;
            }
            if (goal->engage) {
                bool output[2] = {_pump->value(0), _pump->value(1)};
                ROS_INFO("Suction pomp status out1:%d, out2:%d", output[0], output[1]);

                if (!output[0]) {
                    feedback.status = goal->NOTHING;
                } else {
                    if (!output[1]) {
                        feedback.status = goal->HALF_COVER;
                    } else {
                        feedback.status = goal->FULL_COVER;
                    }
                }
                _server.publishFeedback(feedback);
                if(feedback.status >= goal->target_area){
                    break;
                }
            } else {
                break;
            }
            _server.publishFeedback(feedback);
            loop_rate.sleep();
        }

        suction_pump::SuctionPumpResult result;
        result.data = feedback.status;
        if (feedback.status >= goal->target_area || !goal->engage) {
            _server.setSucceeded(result);
        } else {
            if (goal->engage) {
                _pump->disable();
            }
            ROS_ERROR("%s: Aborted", _action_name.c_str());
            _server.setAborted(result);
        }
    }

private:
    std::unique_ptr<Pump> _pump;
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
