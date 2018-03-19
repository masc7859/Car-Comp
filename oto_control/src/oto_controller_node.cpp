#include <oto_control/OtoController.h>

int main(int argc,char**argv)
{
    ros::init(argc, argv, "oto_control");

    OtoController controller;
    if(!controller.initialize()) {
        ROS_INFO("Failed to initialize ros_oto_controller_node");
        return EXIT_FAILURE;
    }else {
        ROS_INFO("Successfully initialized ros_oto_controller_node");
    }

    ros::Rate rate(controller.get_rate_hz());

	  while(ros::ok()) {
      controller.decide_yaw();
      controller.decide_vel();

	    ros::spinOnce();
      rate.sleep();
	  }

	controller.~OtoController();

	ROS_INFO("Exited ros_oto_controller_node");

	return EXIT_SUCCESS;
}
