#pragma once

#include <oto_control/SensorStateList.h>
#include <oto_control/MotorStateList.h>
#include <oto_control/MotorCommand.h>

#include <ros/ros.h>
#include <vector>
#include <map>
#include <angles/angles.h>
#include <string>

using namespace std;

struct ir_data {
  string name;
  float voltage;
};

struct motor_data {
  string name;
  int pulse;
  float radians;
  float degrees;
};

class OtoController {
    private:
        ros::NodeHandle n;
        ros::Subscriber sensor_sub;
        ros::Subscriber motor_sub;
        ros::Publisher motor_pub;
        //ros::Rate rate;

        int desired_distance;
        int rate_hz;

    public:
        ir_data latest_ir_data[2];
        motor_data latest_motor_state[2];
        oto_control::MotorStateList motor_state_list;

        OtoController();
        ~OtoController();
        double get_rate_hz();
        bool initialize();
        void sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg);
        void motor_state_callback(const oto_control::MotorStateList::ConstPtr& msg);
        void publish_motor_command();
};
