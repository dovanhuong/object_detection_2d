# object_detection_2d
This source code to take object detection from 2D and point out the position in 3D coordinate of object from RealSense D435 camera under ROS platform.
By dint of HSV algorithm to filter single color from camera stream for detect object. If you have any futher advice or suggestion, kindly send me an email: dvhbkhn@gmail.com.
I'm willing to disscuss and develop from this.
Process to use this code: 
 $ catkin_make 
 $ roslaunch realsense2_camera rs_rgbd.launch
 $rosrun opencv_object_tracking object_filter
 

