#include <oto_control/OtoController.h>

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

OtoController::OtoController()
{
    int x = 0;
}

OtoController::~OtoController()
{
    ROS_INFO("Shutting down...");
}

void OtoController::sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg) {
    OtoController::latest_ir_data[0].name = msg->sensor_states[0].name;
    OtoController::latest_ir_data[0].voltage = msg->sensor_states[0].voltage;
    OtoController::latest_ir_data[1].name = msg->sensor_states[1].name;
    OtoController::latest_ir_data[1].voltage = msg->sensor_states[1].voltage;
    ROS_INFO("%f",OtoController::latest_ir_data[1].voltage);

    this->sensor_interpret();
}

void OtoController::motor_state_callback(const oto_control::MotorStateList::ConstPtr& msg) {
    OtoController::latest_motor_state[0].name = msg->motor_states[0].name;
    OtoController::latest_motor_state[0].pulse = msg->motor_states[0].pulse;
    OtoController::latest_motor_state[0].radians = msg->motor_states[0].radians;
    OtoController::latest_motor_state[0].degrees = msg->motor_states[0].degrees;
    ROS_INFO("motor_state 1:%d",OtoController::latest_motor_state[0].pulse);

    OtoController::latest_motor_state[1].name = msg->motor_states[1].name;
    OtoController::latest_motor_state[1].pulse = msg->motor_states[1].pulse;
    OtoController::latest_motor_state[1].radians = msg->motor_states[1].radians;
    OtoController::latest_motor_state[1].degrees = msg->motor_states[1].degrees;
    ROS_INFO("steering_state 2:%lf",OtoController::latest_motor_state[1].degrees);

    OtoController::motor_plant_msg.data = OtoController::latest_motor_state[0].pulse;
    OtoController::steering_plant_msg.data = OtoController::latest_motor_state[1].degrees;

}

void OtoController::imu_orientation_callback(const ImuMsg::ConstPtr& imu_msg) {
    double q0,q1,q2,q3;

    //geometry_msgs::Vector3Stamped ;

    q0 = imu_msg->orientation.w;
    q1 = imu_msg->orientation.x;
    q2 = imu_msg->orientation.y;
    q3 = imu_msg->orientation.z;
    OtoController::x_accel = imu_msg->linear_acceleration.x;
    OtoController::y_accel = imu_msg->linear_acceleration.y;

    //tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
    tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(OtoController::roll, OtoController::pitch, OtoController::yaw);
    ROS_INFO("yaw: %lf", OtoController::yaw);
    ROS_INFO("x accel: %lf", OtoController::x_accel);

}

void OtoController::publish_motor_command(oto_control::MotorCommand motor_command) {
    double position = -.2;
    double pose = deg_to_rad(position);
    ROS_INFO("steering command in rad %lf",pose);
    motor_command.joint_name = "drive";
    //motor_command.position = deg_to_rad(0.0);
    motor_command.position = position;
    motor_pub.publish(motor_command);
}

void OtoController::steering_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    oto_control::MotorCommand motor_command;
    OtoController::steering_effort_msg.data = msg->data;
    this->publish_motor_command(motor_command);
}

void OtoController::motor_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    oto_control::MotorCommand motor_command;
    OtoController::motor_effort_msg.data = msg->data;
    this->publish_motor_command(motor_command);
}

void OtoController::publish_steering_setpoint() {
    //OtoController::steering_setpoint_msg.data = 0.0;
    //steering_plant_pub.publish(OtoController::steering_setpoint_msg);
}

void OtoController::publish_motor_setpoint() {
    //motor_plant_pub.publish(OtoController::motor_setpoint_msg);
}

bool OtoController::initialize()
{
    //pololu
    sensor_sub = n.subscribe("pololu/sensor_states", 1, &OtoController::sensor_state_callback, this);
    motor_sub = n.subscribe("pololu/motor_states", 1, &OtoController::motor_state_callback, this);
    motor_pub = n.advertise<oto_control::MotorCommand>("pololu/command", 1);

    //pid
    steering_plant_pub = n.advertise<std_msgs::Float64>("steering_plant", 1);
    steering_setpoint_pub = n.advertise<std_msgs::Float64>("steering_setpoint", 1);
    steering_effort_sub = n.subscribe("steering_effort", 1, &OtoController::steering_effort_callback, this);

    steering_plant_pub = n.advertise<std_msgs::Float64>("motor_plant", 1);
    steering_setpoint_pub = n.advertise<std_msgs::Float64>("motor_setpoint", 1);
    steering_effort_sub = n.subscribe("motor_effort", 1, &OtoController::motor_effort_callback, this);

    //imu
    imu_orientation_sub = n.subscribe("imu/data", 1, &OtoController::imu_orientation_callback, this);

    //this block is temporary
    OtoController::motor_command.joint_name = "steering";
    OtoController::motor_command.position = 1501;
    OtoController::motor_command.speed = 0;
    OtoController::motor_command.acceleration = 0;

    //setup configuration
    OtoController::cfg.cruise_setpoint = 150.0;
    OtoController::cfg.min_turn_distance = 250.0;
    OtoController::turn_flag = false;
    OtoController::turn_state = false;

    bool success = true;
    ROS_INFO("Initialized OtoController");

    return success;
}

void OtoController::sensor_interpret(){
    double distance_plant_comb;

    //distance to wall from each sensor
    OtoController::distance_plant_f = pow(OtoController::latest_ir_data[0].voltage, -3.348) * sqrt(2.0)/2.0 * 7.817 * pow(10.0,10.0) + 34.18;
    OtoController::distance_plant_r = pow(OtoController::latest_ir_data[1].voltage, -3.348) * 7.817 * pow(10.0,10.0) + 34.18;

    if(OtoController::distance_plant_f >= OtoController::cfg.min_turn_distance){
        if(OtoController::turn_flag){
            OtoController::turn_flag_confidence = max(OtoController::turn_flag_confidence,
                (OtoController::distance_plant_f - OtoController::cfg.min_turn_distance) / 400);
        }
        else{
            OtoController::turn_flag = true;
            OtoController::turn_flag_confidence = (OtoController::distance_plant_f - OtoController::cfg.min_turn_distance) / 400;
        }
    }

    if(OtoController::distance_plant_r >= OtoController::cfg.min_turn_distance){
        //cant turn immediately, need some way of telling for sure
        OtoController::turn_state = true;
    }

}

double OtoController::get_rate_hz()
{
    //get this from params
    double rate_hz = 10;
    return rate_hz;
}
