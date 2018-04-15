#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double deg_to_rad(double angle){
    double angle_rad = angle * M_PI / 180;
    return angle_rad;
}
