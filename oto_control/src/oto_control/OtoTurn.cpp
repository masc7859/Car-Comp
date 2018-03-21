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

void OtoController::TurnState::turn(){

}

void OtoController::TurnState::sensor_interpret(){
	//option 1: use yaw, check if we have turned 90 degrees yet.
	if (parent_controller->yaw <= final_yaw){
		parent_controller->state = CRUISE;
	}
  /*
	//option 2: use ir sensors, turn until paralell to a wall (withen error bound turh_threshold):
	ir_f_distance = pow(parent_controller->latest_ir_data[0].voltage, -3.348) * sqrt(2.0)/2.0 * 7.817 * pow(10.0,10.0) + 34.18;
	if_r_distance = pow(parent_controller->latest_ir_data[1].voltage, -3.348) * 7.817 * pow(10.0,10.0) + 34.18;

	infinity_threshold = parent_controller->cfg.min_turn_distance; //really just a value to not do math with absurdly large sensor data
	turn_threshold = 10; //10 cm for now, replace once testing done

	if ((ir_f_distance <= threshold) && (ir_r_distance <= threshold)){
		if (abs(ir_f_distance - ir_r_distance)) < turn_threshold{
			//we are paralell(ish) to a wall
			parent_controller->state = CRUISE;
		}
	}
	//note option 2 has a problem if we turn too fast, rear sensor distance reading will grow to inf, mitigate with following:
	if (ir_r_distance > infinity_threshold){
		//TURN SLOWER not really sure what to modify here
	}

	//option 3: ignore yaw, use magnometer data (which may not drift as much?)
	//not sure how to implement, or if it is a good idea, would require testing
  */
}

OtoController::TurnState::~TurnState()
{
    ROS_INFO("Leaving Turn State");
}

double deg_to_rad(double angle){
    double angle_rad = angle * M_PI / 180;
    return angle_rad;
}
