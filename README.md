# object_detection_2d
This source code to take object detection from 2D and point out the position in 3D coordinate of object from RealSense D435 camera under ROS platform.<br>
By dint of HSV algorithm to filter single color from camera stream for detect object. <br>If you have any futher advice or suggestion, kindly send me an email: vanhuong.robotics@gmail.com <br>
I'm willing to disscuss and develop from this.<br>
Process to use this code: <br>
 $ catkin_make <br>
 $ roslaunch realsense2_camera rs_rgbd.launch <br>
 $rosrun opencv_object_tracking object_filter
 

