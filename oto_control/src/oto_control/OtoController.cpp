#include <oto_control/OtoController.h>

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

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
    ROS_INFO("in callback");
    OtoController::latest_ir_data[0].name = msg->sensor_states[0].name;
    OtoController::latest_ir_data[0].voltage = msg->sensor_states[0].voltage;
    ROS_INFO("%f",OtoController::latest_ir_data[0].voltage);
}

void OtoController::motor_state_callback(const oto_control::MotorStateList::ConstPtr& msg) {
    ROS_INFO("in callback");
    OtoController::latest_motor_state[0].name = msg->motor_states[0].name;
    OtoController::latest_motor_state[0].pulse = msg->motor_states[0].pulse;
    OtoController::latest_motor_state[0].radians = msg->motor_states[0].radians;
    OtoController::latest_motor_state[0].degrees = msg->motor_states[0].degrees;
    ROS_INFO("%d",OtoController::latest_motor_state[0].pulse);

    OtoController::plant_state.data = OtoController::latest_motor_state[0].degrees;
}

void OtoController::publish_motor_command() {
    motor_pub.publish(OtoController::motor_command);
}

void OtoController::steering_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    OtoController::steering_effort.data = msg->data;
}

void OtoController::publish_steering_setpoint(double test) {
    OtoController::plant_test.data = test;
    steering_plant_pub.publish(OtoController::plant_test);
}

void OtoController::publish_steering_plant(double test) {
    OtoController::plant_test.data = test;
    steering_setpoint_pub.publish(OtoController::plant_test);
}

bool OtoController::initialize()
{
    //for pololu
    sensor_sub = n.subscribe("pololu/sensor_states", 1, &OtoController::sensor_state_callback, this);
    motor_sub = n.subscribe("pololu/motor_states", 1, &OtoController::motor_state_callback, this);
    motor_pub = n.advertise<oto_control::MotorCommand>("pololu/command", 1);

    //pid
    steering_plant_pub = n.advertise<std_msgs::Float64>("steering_plant", 1);
    steering_setpoint_pub = n.advertise<std_msgs::Float64>("steering_setpoint", 1);
    steering_effort_sub = n.subscribe("steering_effort", 1, &OtoController::steering_effort_callback, this);

    //this block is temporary
    OtoController::desired_distance = 150;
    OtoController::motor_command.joint_name = "steering";
    OtoController::motor_command.position = 1501;
    OtoController::motor_command.speed = 0;
    OtoController::motor_command.acceleration = 0;

    bool success = true;
    ROS_INFO("Initialized OtoController");

    return success;
}

double OtoController::get_rate_hz()
{
    //get this from params
    double rate_hz = 10;
    return rate_hz;
}
