#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpKeyPoint.h>
#include <ros/ros.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/robot/vpSimulatorCamera.h>

vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
double tagSize = 0.05;
float quad_decimate = 2;
int nThreads = 1;
std::string intrinsic_file = "";
std::string camera_name = "";
bool display_tag = false;
int color_id = -1;
unsigned int thickness = 2;
bool z_aligned = false;
vpCameraParameters cam;
vpImage<unsigned char> I, Idisp, Imot;
const std::string image_file = "/home/tsuchida/image/ig_2.jpg";

void CB(const sensor_msgs::ImageConstPtr&);
void camera_CB(const sensor_msgs::CameraInfoConstPtr&);
int cb_count = 0;
vpDetectorAprilTag detector(tagFamily);
std::vector<vpHomogeneousMatrix> cMo_vec;
vpHomogeneousMatrix cMo;
std::vector<vpFeaturePoint> p(4);
std::vector<vpPoint> point(4);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual servo");
    ros::NodeHandle nh_;
   
    cam.initPersProjWithoutDistortion(476, 476, 400, 300);
    vpImageIo::read(Imot, image_file);
    Idisp.resize(Imot.getHeight(), Imot.getWidth());
    Idisp.insert(Imot, vpImagePoint(0, 0));
    vpDisplayOpenCV d(Idisp, 0, 0, "Matching keypoints");
    vpDisplay::display(Idisp);
    vpDisplay::flush(Idisp);
    
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagQuadDecimate(quad_decimate);
    
    ros::Subscriber camera_sub = nh_.subscribe<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 1000, camera_CB);
    ros::Subscriber image_sub = nh_.subscribe<sensor_msgs::Image>("/usb_cam/image_raw/", 1000, CB);
    ros::spin();
    if (vpDisplay::getClick(Idisp, false))
        vpDisplay::getClick(Idisp);
    return 0;
}

void camera_CB(const sensor_msgs::CameraInfoConstPtr& msg)
{
    cam = visp_bridge::toVispCameraParameters(*msg);
}

void CB(const sensor_msgs::ImageConstPtr& msg)
{
    I = visp_bridge::toVispImage(*msg);
    Idisp.insert(I, vpImagePoint(0, 0));
    vpDisplay::display(Idisp);
    cb_count++;
    detector.detect(I, tagSize, cam, cMo_vec);
    cMo = cMo_vec[cMo_vec.size() - 1];
    std::vector<vpImagePoint> corners = detector.getPolygon(0);

   /* if (cb_count == 1)
    {
        std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
        v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
        for (size_t i = 0; i < 2; i++) {
            v_cdMc[i] = 
        }
    }*/
    for (size_t i = 0; i < corners.size(); i++) {
        vpFeatureBuilder::create(p[i], cam, corners[i]);
        vpColVector cP;
        point[i].changeFrame(cMo, cP);
        p[i].set_Z(cP[2]);
        std::cout << "feature " << i << "x : " << p[i].get_x() << " y : " << p[i].get_y() << " z : " << p[i].get_Z() << std::endl;      
    }
    for (int i = 0; i < p.size(); i++) {
        std::stringstream ss;
        vpImagePoint ip;
        ss << i;
        vpMeterPixelConversion::convertPoint(cam, p[i].get_x(), p[i].get_y(), ip);
        vpDisplay::displayText(Idisp, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
        vpDisplay::displayCross(Idisp, ip, 14, vpColor::red, 4);
    }
    vpDisplay::flush(Idisp);
}

