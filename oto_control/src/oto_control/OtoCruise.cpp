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
    //sensor_interpret();
    //parent_controller->steering_setpoint_msg.data = 1500;
    //parent_controller->steering_plant_msg.data = 1500;
    parent_controller->publish_steering_setpoint();
}

void OtoController::CruiseState::decide_yaw(){
    parent_controller->steering_effort_msg.data;

    motor_command.joint_name = "steering";
    motor_command.position = 1500;
    parent_controller->publish_motor_command(motor_command);
}

void OtoController::CruiseState::decide_vel(){
    //set motor_setpoint based on confidence
    //parent_controller->motor_setpoint_msg.data = 1500;
    parent_controller->publish_motor_setpoint();
}

void OtoController::CruiseState::sensor_interpret(){
    double distance_plant_comb;

    ROS_INFO("actual distance (front): %lf", parent_controller->distance_plant_f);
    ROS_INFO("actual distance (rear): %lf", parent_controller->distance_plant_r);

    //we want plant_f - plant_r = 0, thats our cruise condition

    if(parent_controller->distance_plant_f >= parent_controller->cfg.min_turn_distance){
        if(turn_flag){
            turn_flag_confidence = max(turn_flag_confidence,
                (parent_controller->distance_plant_f - parent_controller->cfg.min_turn_distance) / (550.0 - parent_controller->cfg.min_turn_distance));
        }
        else{
            turn_flag = true;
            turn_flag_confidence = (parent_controller->distance_plant_f - parent_controller->cfg.min_turn_distance) / (550 - parent_controller->cfg.min_turn_distance);
        }
    }

    if(parent_controller->distance_plant_r >= parent_controller->cfg.min_turn_distance){
        //cant turn immediately, need some way of telling for sure
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
