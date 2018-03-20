#include <oto_control/OtoController.h>

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

OtoController::TurnState::TurnState()
{
    ROS_INFO("Moving to Turn State");
}

bool OtoController::TurnState::initialize(OtoController* controller){
    TurnState::parent_controller = controller;
    TurnState::init_yaw = parent_controller->yaw;
    TurnState::final_yaw = parent_controller->yaw - (M_PI / 2);
}

OtoController::TurnState::~TurnState()
{
    ROS_INFO("Leaing Turn State");
}

double deg_to_rad(double angle){
    double angle_rad = angle * M_PI / 180;
    return angle_rad;
}
