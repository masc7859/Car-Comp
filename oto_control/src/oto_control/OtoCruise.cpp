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
    double cur_distance_plant = parent_controller->steering_plant_msg.data;

    sensor_interpret(cur_distance_plant);
    parent_controller->steering_setpoint_msg.data = parent_controller->cfg.cruise_setpoint; //will be getting from
    parent_controller->publish_steering_setpoint();

    decide_vel();
    decide_yaw();
}

void OtoController::CruiseState::decide_yaw(){ //bad name, change
    motor_command.joint_name = "steering";
    //motor_command.position = deg_to_rad(10.);
    //motor_command.position = parent_controller->steering_effort_msg.data + deg_to_rad(5.);
    motor_command.position = parent_controller->steering_effort_msg.data;
    parent_controller->publish_motor_command(motor_command);
}

void OtoController::CruiseState::decide_vel(){
    //set motor_setpoint based on confidence
    motor_command.joint_name = "drive";
    motor_command.position = MAX_SPEED_PW_F * 0.59;
    parent_controller->publish_motor_command(motor_command);

}

void OtoController::CruiseState::sensor_interpret(double cur_distance_plant){
    /*
    parent_controller->debug_msg.data = "actual distance (front): " + to_string(parent_controller->distance_plant_front);
    parent_controller->debug_pub.publish(parent_controller->debug_msg);
    parent_controller->debug_msg.data = "actual distance (rear): " + to_string(parent_controller->distance_plant_rear);
    parent_controller->debug_pub.publish(parent_controller->debug_msg);
    */

    /*
      if(cur_distance_plant > parent_controller->cfg.min_turn_distance){
          bool x = false;
          parent_controller->steering_pid_enable(x);
          parent_controller->turn_init_yaw = parent_controller->yaw;
          parent_controller->state = TURN;
  	}
  	*/
    ROS_INFO("change in distance: %lf", abs(cur_distance_plant - last_distance_plant));
    if (abs(cur_distance_plant - last_distance_plant) > 50.0){
      if (in_doorway = false) {
        in_doorway = true;
        parent_controller->cfg.cruise_setpoint = parent_controller->cfg.cruise_setpoint + 90.0;
      }
      else{
        parent_controller->cfg.cruise_setpoint = parent_controller->cfg.cruise_setpoint - 90.0;
        in_doorway = false;
        parent_controller->doorways++;
        ROS_INFO("exited doorway: %d",parent_controller->doorways);
      }
    }
    last_distance_plant = cur_distance_plant;
}

bool OtoController::CruiseState::initialize(OtoController* controller){
    parent_controller = controller;
    turn_flag = false;
    last_distance_plant = parent_controller->steering_plant_msg.data;
    in_doorway = true;

    bool success = true;
    ROS_INFO("Initialized Cruise State");
    return success;
}

OtoController::CruiseState::~CruiseState()
{
    ROS_INFO("Destructing Cruise State");
}
