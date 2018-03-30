#include <camera/Camera.h>
#include <errno.h>

int main(int argc,char**argv)
{
    ros::init(argc, argv, "camera_control");
    CameraController controller;

    if(!controller.Initialize()) {
        ROS_INFO("Failed to initialize camera_node");
        return EXIT_FAILURE;
    }else {
        ROS_INFO("Successfully initialized camera_node");
    }

    ros::Rate rate(10.0);
    usleep(1000*1000);

	  while(ros::ok()) {
      //do something

	    ros::spinOnce();
      rate.sleep();
    }
	ROS_INFO("Exited camera_node");

	return EXIT_SUCCESS;
}
