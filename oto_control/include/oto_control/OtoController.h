#pragma once

#include <oto_control/SensorStateList.h>
#include <oto_control/MotorStateList.h>
#include <oto_control/MotorCommand.h>

#include <ros/ros.h>
#include <vector>
#include <map>
#include <angles/angles.h>
#include <string>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;

struct ir_data {
  string name;
  double voltage;
};

struct motor_data {
  string name;
  int pulse;
  double radians;
  double degrees;
};

struct configuration {
  double cruise_setpoint;
  double min_turn_distance;
};

class OtoController {
    typedef sensor_msgs::Imu              ImuMsg;

    private:
        ros::NodeHandle n;
        ros::Subscriber sensor_sub;
        ros::Subscriber motor_sub;
        ros::Publisher motor_pub;

        ros::Publisher steering_plant_pub;
        ros::Publisher steering_setpoint_pub;
        ros::Subscriber steering_effort_sub;

        ros::Publisher motor_plant_pub;
        ros::Publisher motor_setpoint_pub;
        ros::Subscriber motor_effort_sub;

        ros::Subscriber imu_orientation_sub;
        //ros::Rate rate;

        int rate_hz;

    public:
        ir_data latest_ir_data[2]; //first front, second rear
        motor_data latest_motor_state[2]; //first motor, second steering
        configuration cfg;
        double steering_plant, distance_plant_f, distance_plant_r, motor_plant;
        bool turning;
        bool turn_flag;
        double turn_flag_confidence;

        oto_control::MotorCommand motor_command;

        std_msgs::Float64 steering_plant_msg;
        std_msgs::Float64 steering_effort_msg;
        std_msgs::Float64 steering_setpoint_msg;
        std_msgs::Float64 motor_plant_msg;
        std_msgs::Float64 motor_effort_msg;
        std_msgs::Float64 motor_setpoint_msg;

        OtoController();
        ~OtoController();
        double get_rate_hz();
        bool initialize();
        void sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg);
        void motor_state_callback(const oto_control::MotorStateList::ConstPtr& msg);
        void publish_motor_command(oto_control::MotorCommand motor_command);
        void steering_effort_callback(const std_msgs::Float64::ConstPtr& msg);
        void motor_effort_callback(const std_msgs::Float64::ConstPtr& msg);
        void imu_orientation_callback(const ImuMsg::ConstPtr& imu_msg);
        void publish_steering_setpoint();
        void publish_steering_plant();
        void publish_motor_setpoint();
        void publish_motor_plant();
        void sensor_interpret();
        void decide_yaw();
        void decide_vel();
};
