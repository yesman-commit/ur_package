#include <ros/ros.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <sensor_msgs/CameraInfo.h>
#include <visp_bridge/image.h>
#include <iostream>

vpCameraParameters cam;

void camera_CB(const sensor_msgs::CameraInfo msg) {
    cam = visp_bridge::toVispCameraParameters(msg);
    cam.printParameters();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_serv3");
    ros::NodeHandle nh;

    ros::Subscriber camera_sub = nh.subscribe<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 1000, camera_CB);
    ros::spin();
    return 0;

}
