#include <camera/Camera.h>

CameraController::~CameraController(){
    camera.stop();
    ROS_INFO("Exiting Camera Controller");
}

bool CameraController::Initialize(){
    ros::NodeHandle np("~");
    devPath = "/dev/video1";
    frame_count = 0;

    //ImgPub = n.advertise<sensor_msgs::Image>("oto_control/image",1);

    np.getParam("rate", rate);
    np.getParam("exposure", exposure);
    np.getParam("brightness", brightness);

    /* USB 3.0 */
    /* 8-bit Greyscale 1280 x 720 60 fps */
    camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 60);

    /* 8-bit Greyscale 1280 x 960 45 fps */
    //camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 45);

    /* 8-bit Greyscale 320 x 240 160 fps */
    //camera.set_format(320, 240, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 160);

    /* 8-bit Greyscale 640 x 480 80 fps */
    //camera.set_format(640, 480, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 80);

    /* USB 2.0 */
    /* 8-bit Greyscale 1280 x 720 30 fps */
    //camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 30);

    /* 8-bit Greyscale 1280 x 960 22.5 fps */
    //camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 2, 45);

    /* 8-bit Greyscale 320 x 240 160 fps */
    //camera.set_format(320, 240, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 160);

    /* 8-bit Greyscale 640 x 480 80 fps */
    //camera.set_format(640, 480, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 80);

    /*
     * get current camera format (image size and frame rate)
     */
    camera.get_current_format(camFormat);

    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();

    ROS_INFO("Connected Camera: dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    //ROS_INFO("----------------- Current format informations -----------------\n");
    //camFormat.print();

    //int brightness = camera.get_control("Brightness");
    //int exposure = camera.get_control("Exposure (Absolute)");

    camera.set_control("Brightness", brightness);
    camera.set_control("Exposure (Absolute)", exposure);

    if (!camera.start()) {
        perror("Failed to start.");
        exit(0);
    }

    bool success = true;
    return success;
}

bool CameraController::PublishImage(){
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;

    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    /* Copy a single frame(image) from camera(oCam-1MGN). This is a blocking function. */
    int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);

    std_msgs::Header header; // empty header
    header.seq = frame_count; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, srcImg);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    ImgPub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

    cam_name = camera.get_dev_name();

    /* If the error occured, restart the camera. */
    if (size == -1) {
        ROS_INFO("error number: %d\n", errno);
      	ROS_INFO("Restarting Camera");
      	camera.stop();
      	camera.start();
    }

    //out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
    //out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    //out_msg.image    = sal_float_image; // Your cv::Mat

    //saliency_img_pub.publish(out_msg.toImageMsg());
    frame_count++;
}
