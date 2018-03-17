#include <oto_control/OtoController.h>
#include <oto_control/SensorStateList.h>

#include <string>
#include <math.h>
#include <ros/ros.h>

using namespace std;

OtoController::OtoController()
{
    int x = 0;
}

OtoController::~OtoController()
{
    ROS_INFO("Shutting down...");
}

void OtoController::sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg) {
    ir_data latest_ir_data_f;
    latest_ir_data_f.name = msg->sensor_states[0].name;
    latest_ir_data_f.voltage = msg->sensor_states[0].voltage;
    ROS_INFO("%f",latest_ir_data_f.voltage);
}

bool OtoController::initialize()
{
    ros::NodeHandle nh("~");
    ros::Subscriber sensor_sub = nh.subscribe("pololu/sensor_states", 1, &OtoController::sensor_state_callback, this);

    bool success = true;
    ROS_INFO("Initialized OtoController");

    return success;
}

double OtoController::get_rate_hz()
{
    //get this from params
    double rate_hz = 10;
    return rate_hz;
}
