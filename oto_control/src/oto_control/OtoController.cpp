#include <oto_control/OtoController.h>

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

OtoController::OtoController() {
    int x = 0;
}

OtoController::~OtoController() {
	oto_control::MotorCommand motor_command;
	motor_command.joint_name = "drive";
    motor_command.position = 0;
    publish_motor_command(motor_command);
    ROS_INFO("Shutting down...");
}

void OtoController::sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg) {
    double distance_plant_comb;
	double distance_front;
	double distance_rear;

	distance_front = pow(msg->sensor_states[FRONT_IR].voltage, -3.348) * 7.817 * pow(10.0,10.0) + 34.18;
	distance_rear = pow(msg->sensor_states[REAR_IR].voltage, -3.348) * 7.817 * pow(10.0,10.0) + 34.18 - 1;

    if(yaw_found){
	  if(abs(distance_front - distance_rear) < 3.0){
	    yaw_zero = yaw;
	  }
	  ROS_INFO("yaw_zero: %lf", yaw_zero);
	  ROS_INFO("yaw_cur: %lf", yaw);
      distance_plant_front = distance_front*cos(yaw_zero - yaw);
      distance_plant_rear = distance_rear*cos(yaw_zero - yaw);
    }
    else{
      distance_plant_front = distance_front;
      distance_plant_rear = distance_rear;
    }

    //merge sensor data with imu
    distance_plant_comb = (distance_plant_front + distance_plant_rear)/2.;
    debug_msg.data = "distance_away from cruising:" + to_string(distance_plant_comb);
    debug_pub.publish(debug_msg);

    filter_ir(distance_plant_comb);
}

void OtoController::filter_ir(double distance_plant_comb) {
    if(ir_count_vec.size() != 10) {
      ir_count_vec.push_back(distance_plant_comb);
    }
    else {
      sort(ir_count_vec.begin(), ir_count_vec.end());
      steering_plant = ir_count_vec[(ir_count_vec.size()/2)-1];
      steering_plant_msg.data = steering_plant;
      if(!isnan(steering_plant_msg.data)){
      	steering_plant_pub.publish(steering_plant_msg);
      	//ROS_INFO("Publishing steering plant of: %lf", steering_plant_msg.data);
      }
      ir_count_vec.clear();
      ir_count_vec.push_back(distance_plant_comb);
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
    //ROS_INFO("Steering state callback radians:%lf",latest_motor_state[STEERING].radians);
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
    if(yaw < 0){
      yaw = -yaw;
    }
    else{
      yaw = M_PI + abs(M_PI-yaw);
    }
    if(!yaw_found){
      yaw_zero = yaw;
      yaw_found = true;
    }
    //ROS_INFO("yaw: %lf", yaw);
    //ROS_INFO("forward/reverse accel: %lf", x_accel);

    /*
    t = ros::Time::now().toSec();
    t_interval = t - t_prev;
    t_prev = t;
    vel_est = vel_est + x_accel * t_interval;
    ROS_INFO("time: %lf",t_interval );
    ROS_INFO("Vel Estimate: %lf", vel_est);
    */

}

void OtoController::vi_slam_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  double q0,q1,q2,q3;
  double x,y,z,roll,pitch,yaw;

  q0 = msg->pose.orientation.x;
  q1 = msg->pose.orientation.y;
  q2 = msg->pose.orientation.z;
  q3 = msg->pose.orientation.w;
  tf2::Matrix3x3(tf2::Quaternion(q0,q1,q2,q3)).getRPY(roll, pitch, yaw);

  x = msg->pose.position.x;
  y = msg->pose.position.y;
  z = msg->pose.position.z;
}

void OtoController::publish_motor_command(oto_control::MotorCommand motor_command) {
    motor_pub.publish(motor_command);
}

void OtoController::steering_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    steering_effort_msg.data = deg_to_rad(msg->data);
    //ROS_INFO("steering_effort in rads: %lf", steering_effort_msg.data);
}

void OtoController::motor_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    motor_effort_msg.data = msg->data;
    //ROS_INFO("motor_effort: %lf", motor_effort_msg.data);

}

void OtoController::publish_steering_setpoint() {
    steering_setpoint_pub.publish(steering_setpoint_msg);
}

void OtoController::publish_motor_setpoint() {
    motor_plant_pub.publish(motor_setpoint_msg);
    //ROS_INFO("motor_setpoint: %lf", motor_setpoint_msg.data);
}

void OtoController::steering_pid_enable(bool x) {
    pid_enable_msg.data = x;
    steering_pid_enable_pub.publish(pid_enable_msg);
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
    steering_pid_enable_pub = n.advertise<std_msgs::Bool>("steering_pid_enable", 1);

    motor_plant_pub = n.advertise<std_msgs::Float64>("motor_plant", 1);
    motor_setpoint_pub = n.advertise<std_msgs::Float64>("motor_setpoint", 1);
    motor_effort_sub = n.subscribe("motor_effort", 1, &OtoController::motor_effort_callback, this);

    //imu
    imu_orientation_sub = n.subscribe("imu/data", 1, &OtoController::imu_callback, this);

    //vi_slam
    vi_slam_pose_sub = n.subscribe("slam/pos", 1, &OtoController::vi_slam_pose_callback, this);

    //debugging broadcast
    debug_pub = n.advertise<std_msgs::String>("oto_control/debug", 1);

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
    cfg.cruise_setpoint = 150.0;
    cfg.min_turn_distance = 800.0;
    filter_ir_count = 0;
    vector<double> ir_count_vec;

    cruiser.initialize(this);
    turner.initialize(this);

    state = CRUISE;
    doorways = 0;
    yaw_found = false;
    t_prev = ros::Time::now().toSec();

    bool success = true;
    ROS_INFO("Initialized OtoController");
    return success;
}

double OtoController::get_rate_hz() {
    //get this from params
    double rate_hz = 50.0;
    return rate_hz;
}
