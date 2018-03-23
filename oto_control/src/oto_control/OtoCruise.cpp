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
    int x = 0;
}

void OtoController::CruiseState::cruise(){
    sensor_interpret();
    parent_controller->steering_setpoint_msg.data = parent_controller->cfg.cruise_setpoint; //will be getting from
    parent_controller->publish_steering_setpoint();

    decide_vel();
    decide_yaw();
}

void OtoController::CruiseState::decide_yaw(){ //bad name, change
    motor_command.joint_name = "steering";
    motor_command.position = parent_controller->steering_effort_msg.data;
    parent_controller->publish_motor_command(motor_command);
}

void OtoController::CruiseState::decide_vel(){
    //set motor_setpoint based on confidence
    motor_command.joint_name = "drive";
    motor_command.position = MAX_SPEED_PW_F * 0.30;
    parent_controller->publish_motor_command(motor_command);

}

void OtoController::CruiseState::sensor_interpret(){
    double distance_plant_comb;

    ROS_INFO("actual distance (left): %lf", parent_controller->distance_plant_left);
    ROS_INFO("actual distance (right): %lf", parent_controller->distance_plant_right);

    if(parent_controller->distance_plant_left >= parent_controller->cfg.min_turn_distance){
        //cant turn immediately, need some way of telling for sure
        parent_controller->turn_init_yaw = parent_controller->yaw;
        parent_controller->state = TURN;
	}

}

bool OtoController::CruiseState::initialize(OtoController* controller){
    parent_controller = controller;
    turn_flag = false;

    bool success = true;
    ROS_INFO("Initialized Cruise State");
    return success;
}

OtoController::CruiseState::~CruiseState()
{
    ROS_INFO("Destructing Cruise State");
}
