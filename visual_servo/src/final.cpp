#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/vision/vpKeyPoint.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fstream>

class VisualServo
{
    
public:
    const std::string detectorName = "ORB";
    const std::string extractorName = "ORB";
    const std::string matcherName = "BruteForce-Hamming";
    vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
    vpImage<vpRGBa> Iin, Imot;
    vpImage<vpRGBa> Idisp;
    ros::NodeHandle nh_;
    ros::Publisher twist_pub = nh_.advertise<geometry_msgs::TwistStamped>("/jog_server/delta_jog_cmds", 1000);
    ros::Subscriber image_sub_1;
    vpServo task;
    vpFeaturePoint p[40], pd[40];
    void CB(const sensor_msgs::ImageConstPtr&);
    VisualServo();
    
    
};
VisualServo::VisualServo()
{
        
    vpImageIo::read(Imot, "/home/tsuchida/image/ig.jpg");
    Idisp.resize(Imot.getHeight(), 2*Imot.getWidth());
    Idisp.insert(Imot, vpImagePoint(0, 0));
    Idisp.insert(Imot, vpImagePoint(0, Imot.getWidth()));
    vpDisplayOpenCV d(Idisp, 0, 0, "Matcheng keypoints whti ORB keypoint");

    vpDisplay::display(Idisp);
    vpDisplay::flush(Idisp);
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    image_sub_1 = nh_.subscribe<sensor_msgs::Image>("/camera/image_raw", 1000, &VisualServo::CB, this);
    vpDisplay::flush(Idisp);
    ROS_INFO("I am ");
    
    }
void VisualServo::CB(const sensor_msgs::ImageConstPtr& msg)
{
    vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);
    keypoint.buildReference(Imot);
    std::ofstream ofs("/home/tsuchida/output_file/vel_1.txt");
    //const std::string PLANNING_GROUP = "manipulator";
    // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::TwistStamped out_cmd;
    out_cmd.header.stamp.sec = 1599662324;
    out_cmd.header.stamp.nsec = 832165956;
    Iin = visp_bridge::toVispImageRGBa(*msg);
    unsigned int npMatch = keypoint.matchPoint(Iin);
    Idisp.insert(Iin, vpImagePoint(0, Imot.getWidth()));
    vpDisplay::display(Idisp);
    vpDisplay::displayLine(Idisp, vpImagePoint(0, Iin.getWidth()), vpImagePoint(Iin.getHeight(), Iin.getWidth()), 
                            vpColor::white, 2);
    vpDisplay::flush(Idisp);
    vpImagePoint iPref, iPcur;
    //  double Z = move_group.getCurrentPose().pose.position.z;
    double Z = 1.0;
    
    for (unsigned int i = 0; i < 20; i++) {
        keypoint.getMatchedPoints(i, iPref, iPcur);
        pd[i].set_xyZ(iPref.get_i(), iPref.get_j(), Z);
        vpDisplay::displayLine(Idisp, iPref, iPcur + vpImagePoint(0, Iin.getWidth()), vpColor::green);
        p[i].set_xyZ(iPcur.get_i(), iPcur.get_j(), Z);
        task.addFeature(p[i], pd[i]);
    }
    vpDisplay::flush(Idisp);
    vpColVector v = task.computeControlLaw();
    out_cmd.twist.linear.x = 0;
    out_cmd.twist.linear.y = v[1] * 1000;
    out_cmd.twist.linear.z = v[0] * 1000;
    out_cmd.twist.angular.x = 0;
    out_cmd.twist.angular.y = 0;
    out_cmd.twist.angular.z = 0;
    twist_pub.publish(out_cmd);
    ofs << v << std::endl;
    ROS_INFO("SHinya");
}  
int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_servo");
    
    VisualServo t;      
    ros::spin();  
    return 0;
}