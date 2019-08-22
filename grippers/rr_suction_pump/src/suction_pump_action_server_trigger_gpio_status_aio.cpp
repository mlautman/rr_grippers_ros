/*
Last Update 20181222, yu.okamoto@rapyuta-robotics.com
*/

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

// #include <rr_hw_interface/gpio/libsoc_gpio.hpp>
#include <rr_hw_interface/gpio/revpi_gpio.hpp>
#include <rr_hw_interface/aio/revpi_aio.hpp>
#include <rr_suction_pump/SuctionPumpAction.h>
#include <rr_suction_pump/suction_pump.hpp>

namespace rapyuta
{

template <class HI_Trigger, class HI_Status, class Config_Trigger, class Config_Status>
class SuctionPumpActionServer
{
public:
    SuctionPumpActionServer(ros::NodeHandle& nh, const std::string& action_name)
            : _server(nh, action_name, boost::bind(&SuctionPumpActionServer::action_cb, this, _1), false)
            , _action_name(action_name), _status_th_num(0)
    {
        int trigger_num = 0;
        int status_num = 0;
        bool outputNormallyOn = false;
        std::vector<std::string> trigger_gpio;
        std::vector<std::string> status_gpio;
        std::string trigger, status;
        int status_th_val;

        nh.getParam("/trigger_num", trigger_num);
        nh.getParam("/status_num", status_num);
        nh.getParam("/output_normally_on", outputNormallyOn);
        for (int i = 0; i < trigger_num; i++) {
            nh.getParam("trigger_gpio" + std::to_string(i), trigger);
            trigger_gpio.push_back(trigger);
        }
        for (int i = 0; i < status_num; i++) {
            nh.getParam("status_gpio" + std::to_string(i), status);
            status_gpio.push_back(status);
        }
        _pump = std::unique_ptr<Pump<HI_Trigger, HI_Status, Config_Trigger, Config_Status, int, bool>>(new Pump<HI_Trigger, HI_Status, Config_Trigger, Config_Status, int, bool>(trigger_gpio, trigger_num, status_gpio, status_num, outputNormallyOn, false));

        nh.getParam("/status_th_num", _status_th_num);
        for (int i = 0; i < _status_th_num; i++) {
            nh.getParam("status_th" + std::to_string(i), status_th_val);
            _status_th.push_back(status_th_val);
        }

    }

    bool init(Config_Trigger _config_trigger, Config_Status _config_status)
    {
        if (_pump->init(_config_trigger, _config_status)) {
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
                feedback.status  = 0;
                int output = _pump->get(0);
                for (int i = 0; i < _status_th_num; i++) {
                    if(_status_th[i] > output){
                        feedback.status = i;
                        break;
                    }
                }
                ROS_INFO("Suction pump status ", feedback.status);
                
                // Publish feedback and set result
                _server.publishFeedback(feedback);
                result.data = feedback.status;

                // If successful, set action to succeeded
                if (result.data >= goal->target_area) {
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

            if (!output[0]) { // if already nothing, reurn success immediately
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
    std::unique_ptr<Pump<HI_Trigger, HI_Status, Config_Trigger, Config_Status, int, bool>> _pump;
    Config_Trigger _config_trigger;
    Config_Status _config_status;
    actionlib::SimpleActionServer<rr_suction_pump::SuctionPumpAction> _server;
    std::string _action_name;
    int _status_th_num;
    std::vector<int> _status_th;

};

} // namespace rapyuta

int main(int argc, char** argv)
{
    ros::init(argc, argv, "suction_pump_action_server");
    ros::NodeHandle nh("~");
    rapyuta::RevPiGpioBoardConfig config_out;
    rapyuta::RevPiAioBoardConfig config_in;
    rapyuta::SuctionPumpActionServer<rapyuta::RevPiGpio, rapyuta::RevPiAio, rapyuta::RevPiGpioBoardConfig, rapyuta::RevPiAioBoardConfig> spas(nh, "suction_pump_action_server");
    if (!spas.init(config_out, config_in)) {
        return 1;
    }
    ros::spin();
    return 0;
}
