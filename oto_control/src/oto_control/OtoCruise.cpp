#include <oto_control/OtoController.h>

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

OtoController::CruiseState::CruiseState()
{
    ROS_INFO("Moving Into Cruise State");
}

void OtoController::CruiseState::decide_yaw(){
    //set steering_setpoint based on ?
    CruiseState::parent_controller->steering_setpoint_msg.data = 1500;
    CruiseState::parent_controller->publish_steering_setpoint();
}

void OtoController::CruiseState::decide_vel(){
    //set motor_setpoint based on confidence
    CruiseState::parent_controller->motor_setpoint_msg.data = 1500;
    CruiseState::parent_controller->publish_motor_setpoint();
}

bool OtoController::CruiseState::initialize(OtoController* controller){
    CruiseState::parent_controller = controller;
}

OtoController::CruiseState::~CruiseState()
{
    ROS_INFO("Leaving Cruise State");
}
