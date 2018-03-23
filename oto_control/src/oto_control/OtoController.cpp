#include <oto_control/OtoController.h>

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

OtoController::OtoController() {
    int x = 0;
}

OtoController::~OtoController() {
    ROS_INFO("Shutting down...");
}

void OtoController::sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg) {
    distance_plant_left = pow(msg->sensor_states[FRONT_IR].voltage, -3.348) * 7.817 * pow(10.0,10.0) + 34.18;
    distance_plant_right = pow(msg->sensor_states[REAR_IR].voltage, -3.348) * 7.817 * pow(10.0,10.0) + 34.18;

    if(distance_plant_right != 0.0) {
        steering_plant_msg.data = distance_plant_right - distance_plant_left;
        steering_plant_pub.publish(steering_plant_msg);
        ROS_INFO("Publishing steering plant of: %lf", steering_plant_msg.data);
    }
}

void OtoController::motor_state_callback(const oto_control::MotorStateList::ConstPtr& msg) {
    latest_motor_state[MOTOR].name = msg->motor_states[MOTOR].name;
    latest_motor_state[MOTOR].pulse = msg->motor_states[MOTOR].pulse;
    latest_motor_state[MOTOR].radians = msg->motor_states[MOTOR].radians;
    latest_motor_state[MOTOR].degrees = msg->motor_states[MOTOR].degrees;
    //ROS_INFO("motor_state:%d",latest_motor_state[MOTOR].pulse);

    latest_motor_state[STEERING].name = msg->motor_states[STEERING].name;
    latest_motor_state[STEERING].pulse = msg->motor_states[STEERING].pulse;
    latest_motor_state[STEERING].radians = msg->motor_states[STEERING].radians;
    latest_motor_state[STEERING].degrees = msg->motor_states[STEERING].degrees;
    ROS_INFO("Steering state callback radians:%lf",latest_motor_state[STEERING].radians);
}

void OtoController::imu_callback(const ImuMsg::ConstPtr& imu_msg) {
    double q0,q1,q2,q3;
    double t;
    double t_interval;

    q0 = imu_msg->orientation.w;
    q1 = imu_msg->orientation.x;
    q2 = imu_msg->orientation.y;
    q3 = imu_msg->orientation.z;
    x_accel = imu_msg->linear_acceleration.x;
    y_accel = imu_msg->linear_acceleration.y;

    tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(roll, pitch, yaw);
    ROS_INFO("yaw: %lf", yaw);
    ROS_INFO("x accel: %lf", x_accel);

    t = ros::Time::now().toSec();
    t_interval = t - t_prev;
    t_prev = t;

    vel_est = vel_est + x_accel * t_interval;
    ROS_INFO("Vel Estimate: %lf", vel_est);

}

void OtoController::publish_motor_command(oto_control::MotorCommand motor_command) {
    motor_pub.publish(motor_command);
}

void OtoController::steering_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    steering_effort_msg.data = deg_to_rad(msg->data);
    ROS_INFO("steering_effort in rads: %lf", steering_effort_msg.data);
}

void OtoController::motor_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    motor_effort_msg.data = msg->data;
    ROS_INFO("motor_effort: %lf", motor_effort_msg.data);

}

void OtoController::publish_steering_setpoint() {
    steering_setpoint_pub.publish(steering_setpoint_msg);
}

void OtoController::publish_motor_setpoint() {
    motor_plant_pub.publish(motor_setpoint_msg);
    ROS_INFO("motor_setpoint: %lf", motor_setpoint_msg.data);
}

bool OtoController::initialize() {
    oto_control::MotorCommand motor_command;

    //pololu
    sensor_sub = n.subscribe("pololu/sensor_states", 1, &OtoController::sensor_state_callback, this);
    motor_sub = n.subscribe("pololu/motor_states", 1, &OtoController::motor_state_callback, this);
    motor_pub = n.advertise<oto_control::MotorCommand>("pololu/command", 1);

    //pid
    steering_plant_pub = n.advertise<std_msgs::Float64>("steering_plant", 1);
    steering_setpoint_pub = n.advertise<std_msgs::Float64>("steering_setpoint", 1);
    steering_effort_sub = n.subscribe("steering_effort", 1, &OtoController::steering_effort_callback, this);

    motor_plant_pub = n.advertise<std_msgs::Float64>("motor_plant", 1);
    motor_setpoint_pub = n.advertise<std_msgs::Float64>("motor_setpoint", 1);
    motor_effort_sub = n.subscribe("motor_effort", 1, &OtoController::motor_effort_callback, this);

    //imu
    imu_orientation_sub = n.subscribe("imu/data", 1, &OtoController::imu_callback, this);

    //set speed(aceleration) and acceleration(jerk)
    motor_command.joint_name = "steering";
    motor_command.position = 0;
    motor_command.speed = 2;
    motor_command.acceleration = 0;
    //this->publish_steering_command(motor_command);

    motor_command.joint_name = "drive";
    motor_command.position = 0;
    motor_command.speed = 5;
    motor_command.acceleration = 0;
    //this->publish_motor_command(motor_command);

    //setup configuration
    cfg.cruise_setpoint = 0.0;
    cfg.min_turn_distance = 550.0;

    cruiser.initialize(this);
    turner.initialize(this);

    state = CRUISE;
    t_prev = ros::Time::now().toSec();

    bool success = true;
    ROS_INFO("Initialized OtoController");
    return success;
}

double OtoController::get_rate_hz() {
    //get this from params
    double rate_hz = 20.0;
    return rate_hz;
}
