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
    parent_controller->steering_setpoint_msg.data = 1500;
    parent_controller->steering_plant_msg.data = 1500;
    parent_controller->publish_steering_setpoint();
}

void OtoController::CruiseState::decide_yaw(){
    //set steering_setpoint based on ?
    parent_controller->steering_setpoint_msg.data = 1500;
    parent_controller->publish_steering_setpoint();
}

void OtoController::CruiseState::decide_vel(){
    //set motor_setpoint based on confidence
    parent_controller->motor_setpoint_msg.data = 1500;
    parent_controller->publish_motor_setpoint();
}

void OtoController::CruiseState::sensor_interpret(){
    double distance_plant_comb;
    double distance_plant_f, distance_plant_r;

    //distance to wall from each sensor
    distance_plant_f = pow(parent_controller->latest_ir_data[FRONT_IR].voltage, -3.348) * sqrt(2.0)/2.0 * 7.817 * pow(10.0,10.0) + 34.18;
    distance_plant_r = pow(parent_controller->latest_ir_data[REAR_IR].voltage, -3.348) * 7.817 * pow(10.0,10.0) + 34.18;

    ROS_INFO("actual distance (front): %lf", distance_plant_f);
    ROS_INFO("actual distance (rear): %lf", distance_plant_r);

    //we want plant_f - plant_r = 0, thats our cruise condition

    if(distance_plant_f >= parent_controller->cfg.min_turn_distance){
        if(turn_flag){
            turn_flag_confidence = max(turn_flag_confidence,
                (distance_plant_f - parent_controller->cfg.min_turn_distance) / 400);
        }
        else{
            turn_flag = true;
            turn_flag_confidence = (distance_plant_f - parent_controller->cfg.min_turn_distance) / 400;
        }
    }

    if(distance_plant_r >= parent_controller->cfg.min_turn_distance){
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
