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

class OtoController {
    private:
        ros::NodeHandle n;
        ros::Subscriber sensor_sub;
        ros::Subscriber motor_sub;
        ros::Publisher motor_pub;
        ros::Publisher steering_plant_pub;
        ros::Publisher steering_setpoint_pub;
        ros::Subscriber steering_effort_sub;
        //ros::Rate rate;

        int desired_distance;
        int rate_hz;

    public:
        ir_data latest_ir_data[2];
        motor_data latest_motor_state[2];
        oto_control::MotorCommand motor_command;
        std_msgs::Float64 plant_state;
        std_msgs::Float64 steering_effort;
        std_msgs::Float64 plant_test;

        OtoController();
        ~OtoController();
        double get_rate_hz();
        bool initialize();
        void sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg);
        void motor_state_callback(const oto_control::MotorStateList::ConstPtr& msg);
        void publish_motor_command();
        void steering_effort_callback(const std_msgs::Float64::ConstPtr& msg);
        void publish_steering_setpoint(double test);
        void publish_steering_plant(double test);
};
