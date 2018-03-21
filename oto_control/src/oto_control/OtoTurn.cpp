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
		
    bool success = true;
    ROS_INFO("Initialized Turn State");
    return success;
}

void OtoController::TurnState::turn(){
	init_yaw = parent_controller->turn_init_yaw;
    final_yaw = parent_controller->turn_init_yaw - (M_PI / 2);
	sensor_interpret();
}

void OtoController::TurnState::sensor_interpret(){
	//option 1: use yaw, check if we have turned 90 degrees yet.
	if (parent_controller->yaw <= final_yaw){
		parent_controller->state = CRUISE;
	}
	//option 2: use ir sensors, turn until paralell to a wall (withen error bound turh_threshold):
	double infinity_threshold, parallel_threshold;

	infinity_threshold = parent_controller->cfg.min_turn_distance; //really just a value to not do math with absurdly large sensor data
	parallel_threshold = 10; //10 cm for now, replace once testing done

	if ((parent_controller->distance_plant_f <= infinity_threshold) && (parent_controller->distance_plant_r <= infinity_threshold)){
		if (abs(parent_controller->distance_plant_f - parent_controller->distance_plant_r) < parallel_threshold){
			//we are parallel(ish) to a wall
			parent_controller->state = CRUISE;
		}
	}
	//note option 2 has a problem if we turn too fast, rear sensor distance reading will grow to inf, mitigate with following:
	if (parent_controller->distance_plant_r > infinity_threshold){
		//TURN SLOWER not really sure what to modify here
		
		motor_command.joint_name = "steering";
		motor_command.position = deg_to_rad(-15);////////// //slower
		parent_controller->publish_motor_command(motor_command);
		
    ROS_INFO("turning at 15/45");
	}
	else{
		motor_command.joint_name = "steering";
		motor_command.position = deg_to_rad(-45);////////// //slower
		parent_controller->publish_motor_command(motor_command);
	
		ROS_INFO("turning at 45/45");}


}

OtoController::TurnState::~TurnState()
{
    ROS_INFO("Leaving Turn State");
}
