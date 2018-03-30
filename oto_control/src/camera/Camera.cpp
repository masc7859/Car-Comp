#include <camera/Camera.h>

using namespace std;

CameraController::CameraController(){
    int x = 10;
}

CameraController::~CameraController(){
    ROS_INFO("Exiting Camera Controller");
}

bool CameraController::Initialize(){
  Withrobot::Camera camera(devPath);

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
  Withrobot::camera_format camFormat;
  camera.get_current_format(camFormat);

  /*
   * Print infomations
   */
  std::string camName = camera.get_dev_name();
  std::string camSerialNumber = camera.get_serial_number();

  ROS_INFO("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
  ROS_INFO("----------------- Current format informations -----------------\n");
  camFormat.print();
  ROS_INFO("---------------------------------------------------------------\n");
  
  bool success = true;
  return success;
}
