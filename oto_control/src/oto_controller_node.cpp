#include <oto_control/OtoController.h>
#include<termios.h>

int khbit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(int state)
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if ( state == 1)
    {
        ttystate.c_lflag &= (~ICANON & ~ECHO); //Not display character
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state == 0)
    {
        ttystate.c_lflag |= ICANON;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

bool keyState(int key) //Use ASCII table
{
    bool pressed;
    int i = khbit(); //Alow to read from terminal
    if (i != 0)
    {
        char c = fgetc(stdin);
        if (c == (char) key)
        {
            pressed = true;
        }
        else
        {
            pressed = false;
        }
    }

    return pressed;
}

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
    usleep(1000*1000);

	 while(ros::ok()) {
     /*
	  	nonblock(1);
      if (keyState(32)) { //32 in ASCII table correspond to Space Bar
        nonblock(0);
        break;
      }
      */
	    switch(controller.state){
        case 0:
          	controller.cruiser.cruise();
          	break;
        case 1:
          	controller.turner.turn();
          	break;
    }

	    ros::spinOnce();
      rate.sleep();
	  }


	controller.~OtoController();

	ROS_INFO("Exited ros_oto_controller_node");
	ros::shutdown();

	return EXIT_SUCCESS;
}
