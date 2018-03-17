#pragma once

#include <oto_control/SensorStateList.h>

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

class OtoController {
    private:
        ros::NodeHandle n;
        ros::Subscriber sensor_sub;
        //ros::Rate rate;

        int rate_hz;

    public:
        OtoController();
        ~OtoController();
        double get_rate_hz();
        bool initialize();
        void sensor_state_callback(const oto_control::SensorStateList::ConstPtr& msg);
};
