Image Pipeline:
Publish images on topic /camera/image_raw:
roslaunch uvccam.launch

To access camera stream: rosrun image_view image_view image:=oto_control/image
			rosrun image_view image_view image:=/camera/image_raw

Calibrate Camera and Produce config yaml:
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.026 image:=/camera/image_raw camera:=/camera

Rectify Image and Publishes to /camera/image_rect if we have topics publishing on /camera/camera_info and /camera/image_raw
ROS_NAMESPACE=camera rosrun image_proc image_proc

orbslam example:
./mono_tum ./../../Vocabulary/ORBvoc.txt TUM1.yaml '/home/maxrebo/Downloads/rgbd_dataset_freiburg1_desk' 

obslam ros:
roslaunch camtest.launch 

orbslam:
rosrun ORB_SLAM2 Mono ${ORBSLAM_2_HOME}/Vocabulary/ORBvoc.txt ${ORBSLAM2_HOME}/Examples/ROS/ORB_SLAM2/camera_conf.yaml

