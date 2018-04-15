#pragma once

#include <ros/ros.h>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <camera/withrobot_camera.hpp>
#include <camera/withrobot_utility.hpp>

class CameraController{
    private:
        ros::NodeHandle n;
        ros::Publisher ImgPub;
        Withrobot::camera_format camFormat;
        const char* devPath;
        int frame_count;
        std::string cam_name;

    public:
        Withrobot::Camera camera;
        double rate;
        int exposure;
        int brightness;


        CameraController(): camera("/dev/video1"){};
        ~CameraController();
        bool Initialize();
        bool PublishImage();

};
