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
    parent_controller = controller;
    init_yaw = parent_controller->yaw;
    final_yaw = parent_controller->yaw - (M_PI / 2);
}

void OtoController::TurnState::sensor_interpret(){

}

OtoController::TurnState::~TurnState()
{
    ROS_INFO("Leaving Turn State");
}

double deg_to_rad(double angle){
    double angle_rad = angle * M_PI / 180;
    return angle_rad;
}
