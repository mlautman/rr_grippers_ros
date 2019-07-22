/*
Last Update 20181222, yu.okamoto@rapyuta-robotics.com
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <rr_hw_interface/gpio/libsoc_gpio.hpp>
#include <rr_suction_pump/SuctionPumpAction.h>
#include <rr_suction_pump/suction_pump.hpp>

namespace rapyuta
{

template <class HI, class Config>
class SuctionPumpActionServer
{
public:
    SuctionPumpActionServer(ros::NodeHandle& nh, const std::string& action_name)
            : _server(nh, action_name, boost::bind(&SuctionPumpActionServer::action_cb, this, _1), false)
            , _action_name(action_name)
    {
        int trigger_num = 0;
        int status_num = 0;
        bool normallyOn = false;
        std::vector<std::string> trigger_gpio;
        std::vector<std::string> status_gpio;
        std::string trigger, status;

        nh.getParam("trigger_num", trigger_num);
        nh.getParam("status_num", status_num);
        nh.getParam("normally_on", normallyOn);
        for (int i = 0; i < trigger_num; i++) {
            nh.getParam("trigger_gpio" + std::to_string(i), trigger);
            trigger_gpio.push_back(trigger);
        }
        for (int i = 0; i < status_num; i++) {
            nh.getParam("status_gpio" + std::to_string(i), status);
            status_gpio.push_back(status);
        }
        _pump = std::unique_ptr<Pump<HI, Config>>(new Pump<HI, Config>(trigger_gpio, trigger_num, status_gpio, status_num, normallyOn));
    }

    bool init()
    {
        if (_pump->init(_config)) {
            _server.start();
            _pump->set(false); // set off
            ROS_INFO("%s: Started", _action_name.c_str());
            return true;
        }
        return false;
    }

    void action_cb(const rr_suction_pump::SuctionPumpGoalConstPtr& goal)
    {
        // Result and feedback
        rr_suction_pump::SuctionPumpResult result;
        rr_suction_pump::SuctionPumpFeedback feedback;
        feedback.status = goal->NOTHING;

        ros::Rate loop_rate(10);
        ros::Time start_time = ros::Time::now();

        // Engage
        if (goal->engage) {
            _pump->set(true);

            // Wait for successful suction
            while ((ros::Time::now() - start_time) < ros::Duration(goal->timeout)) {

                // Check for preempted action
                if (_server.isPreemptRequested() || !ros::ok()) {
                    ROS_INFO("%s: Preempted", _action_name.c_str());
                    _server.setPreempted();
                    return;
                }

                // Get current suction status
                bool output[2] = {_pump->get(0), _pump->get(1)};
                ROS_INFO("Suction pump status out1:%d, out2:%d", output[0], output[1]);
                feedback.status = goal->NOTHING;
                if (output[0]){
                    feedback.status = goal->HALF_COVER;
                }
                if (output[0] && output[1]){
                    feedback.status = goal->FULL_COVER;
                }

                // Publish feedback and set result
                _server.publishFeedback(feedback);
                result.data = feedback.status;

                // If successful, set action to succeeded
                if (result.data >= goal->target_area)
                {
                    _server.setSucceeded(result);
                    return;
                }
                loop_rate.sleep();
            }

            // Action failed
            _pump->set(false);
            ROS_ERROR("%s: Aborted", _action_name.c_str());
            _server.setAborted(result);
        }

        // Disengage
        if (!goal->engage) {

            bool output[2] = {_pump->get(0), _pump->get(1)};
            ROS_INFO("Suction pump status out1:%d, out2:%d", output[0], output[1]);
            result.data = goal->NOTHING;

            if(!output[0]){ //if already nothing, reurn success immediately
                _pump->set(false);
                _server.setSucceeded(result);
                return;
            }

            _pump->set(false);
            while ((ros::Time::now() - start_time) < ros::Duration(goal->timeout)) {
            }
            _server.setSucceeded(result);
        }
    }

private:
    std::unique_ptr<Pump<HI, Config>> _pump;
    Config _config;
    actionlib::SimpleActionServer<rr_suction_pump::SuctionPumpAction> _server;
    std::string _action_name;
};

} // namespace rapyuta

int main(int argc, char** argv)
{
    ros::init(argc, argv, "suction_pump_action_server");
    ros::NodeHandle nh;
    rapyuta::SuctionPumpActionServer<rapyuta::LibsocGpio, rapyuta::LibsocBoardConfig> spas(nh, "suction_pump_action_server");
    if (!spas.init()) {
        return 1;
    }
    ros::spin();
    return 0;
}
