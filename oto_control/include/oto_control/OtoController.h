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

#define MAX_STEERING_ANGLE    45  //left
#define MIN_STEERING_ANGLE    -45 //right
#define MAX_SPEED_PW_F   -0.437
#define MAX_SPEED_PW_R    0.437
#define CRUISE    0
#define TURN    1
#define STEERING    1
#define MOTOR   0
#define FRONT_IR   0
#define REAR_IR   1

double deg_to_rad(double angle);

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
    typedef sensor_msgs::Imu    ImuMsg;

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
        class CruiseState {
            private:
            public:
              double init_yaw;
              double final_yaw;
              double steering_plant, motor_plant;
              bool turn_flag;
              double turn_flag_confidence;
              OtoController* parent_controller;
              oto_control::MotorCommand motor_command;

              CruiseState();
              ~CruiseState();
              bool initialize(OtoController* parent_controller);
              void decide_yaw();
              void decide_vel();
              void sensor_interpret();
              void cruise();
        } cruiser;

        class TurnState {
            private:
            public:
              double init_yaw;
              double final_yaw;
		          double infinity_threshold, parallel_threshold;
              double steering_plant, motor_plant;
              OtoController* parent_controller;
              oto_control::MotorCommand motor_command;

              TurnState();
              ~TurnState();
              bool initialize(OtoController* controller);
              void sensor_interpret();
              void turn();
        } turner;

        ir_data latest_ir_data[2]; //first front, second rear
        motor_data latest_motor_state[2]; //first motor, second steering
        configuration cfg;

        int state;
        double turn_init_yaw;
        double roll,pitch,yaw; //in rad
        double x_accel, y_accel; //in m/s
        double vel_est;
        float t_prev;
        double distance_plant_left, distance_plant_right;

        std_msgs::Float64 steering_plant_msg, steering_effort_msg, steering_setpoint_msg,
                          motor_plant_msg, motor_effort_msg, motor_setpoint_msg;

        OtoController();
        ~OtoController();
        double get_rate_hz();
        bool initialize();
        void sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg);
        void motor_state_callback(const oto_control::MotorStateList::ConstPtr& msg);
        void publish_motor_command(oto_control::MotorCommand motor_command);
        void steering_effort_callback(const std_msgs::Float64::ConstPtr& msg);
        void motor_effort_callback(const std_msgs::Float64::ConstPtr& msg);
        void imu_callback(const ImuMsg::ConstPtr& imu_msg);
        void publish_steering_setpoint();
        void publish_steering_plant();
        void publish_motor_setpoint();
        void publish_motor_plant();
        void decide_yaw();
        void decide_vel();
};
