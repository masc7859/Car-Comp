#include <oto_control/OtoController.h>

#include <string>
#include <math.h>
#include <ros/ros.h>

OtoController::OtoController()
{
    int x = 0;
}

OtoController::~OtoController()
{
    ROS_INFO("Shutting down...");
}

bool OtoController::initialize()
{
    ros::NodeHandle nh("~");

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
