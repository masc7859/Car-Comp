#pragma once

#include <ros/ros.h>
#include <vector>
#include <map>
#include <angles/angles.h>
#include <string>

class OtoController
{
    private:
        ros::NodeHandle n;
        //ros::Rate rate;

        int rate_hz;

    public:
        OtoController();
        ~OtoController();
        double get_rate_hz();
        bool initialize();
};
