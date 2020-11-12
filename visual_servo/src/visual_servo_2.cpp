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
#include <visp3/core/vpImageDraw.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <fstream>
#include <cmath>
#include <limits>

vpImage<unsigned char> Iin, Imot;
vpImage<unsigned char> Idisp;
const std::string mask_image = "/home/tsuchida/image/mask.jpg";
vpColVector v;
vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
double opt_tagSize = 0.05;
int opt_quad_decimate = 2;
vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
vpDetectorAprilTag detector(tagFamily);
vpDetectorAprilTag mask_detect(tagFamily);
std::vector<vpHomogeneousMatrix> cMo_vec;
std::vector<vpHomogeneousMatrix> cdMo_vec;
vpServo task;
std::vector<vpFeaturePoint> p(4), pd(4); 
std::ofstream ofs("/home/tsuchida/output_file/vel_1.txt");
std::ofstream coor("/home/tsuchida/output_file/coordinate.txt");
std::ofstream counts("/home/tsuchida/output_file/count.txt");
std::ofstream error("/home/tsuchida/output_file/error.txt");
ros::Publisher twist_pub, image_pub;
vpCameraParameters cam;
vpHomogeneousMatrix cMo, oMo;
vpHomogeneousMatrix cdMo( vpTranslationVector(0, 0, opt_tagSize * 3), // 3 times tag with along camera z axis
                               vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );
void CB(const sensor_msgs::ImageConstPtr&);
void camera_CB(const sensor_msgs::CameraInfoConstPtr&);
static bool first_time = true;
std::vector<vpPoint> point(4);
int cB_count = 0;
int count = 0;

double first_step = 0.2;
double second_step = 0.08;
double goal;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_servo");
    ros::NodeHandle nh_;
    
    cam.initPersProjWithoutDistortion(577, 586, 309, 158);
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    vpAdaptiveGain lamda(1.5, -0.4, 30);
    
    vpImageIo::read(Imot, mask_image);
    Idisp.resize(Imot.getHeight(), Imot.getWidth());
    Idisp.insert(Imot, vpImagePoint(0, 0));
    vpDisplayOpenCV d(Idisp, 0, 0, "Matcheng keypoints whti ORB keypoint");
    vpDisplay::display(Idisp);
    vpDisplay::flush(Idisp);
     
    /*point[0].setWorldCoordinates(-opt_tagSize/2., -opt_tagSize/2., 0);
    point[1].setWorldCoordinates( opt_tagSize/2., -opt_tagSize/2., 0);
    point[2].setWorldCoordinates( opt_tagSize/2.,  opt_tagSize/2., 0);
    point[3].setWorldCoordinates(-opt_tagSize/2.,  opt_tagSize/2., 0);*/

    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);
    mask_detect.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    mask_detect.setAprilTagQuadDecimate(opt_quad_decimate);
    task.setLambda(-0.5);
    for (int i = 0; i < p.size(); i++) {
        task.addFeature(p[i], pd[i]);
    }
    v.resize(6);
    static bool first_time = true;
    
   
    
    twist_pub = nh_.advertise<geometry_msgs::TwistStamped>("/jog_server/delta_jog_cmds", 1000); 
    image_pub = nh_.advertise<sensor_msgs::Image>("/original/image_raw", 1000);
    ros::Subscriber camera_sub = nh_.subscribe<sensor_msgs::CameraInfo>("camera/camera_info", 1000, camera_CB);
    ros::Subscriber image_sub_1 = nh_.subscribe<sensor_msgs::Image>("camera/image_raw", 1000, CB);
   
    ROS_INFO("count %i", count++); 
    counts << "count is " << count << std::endl;  
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
    geometry_msgs::TwistStamped out_cmd;
    /*out_cmd.header.stamp.sec = 1599662324;
    out_cmd.header.stamp.nsec = 832165956;*/
    out_cmd.header.stamp = ros::Time::now();
    Iin = visp_bridge::toVispImage(*msg);
    Idisp.insert(Iin, vpImagePoint(0, 0));
    counts << "CB_count is " << cB_count++ << std::endl;

    if (cB_count <= 1) {
         std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
        v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
        for (size_t i = 0; i < 2; i++) {
            v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
        }
        if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc[1].getThetaUVector().getTheta())) {
            oMo = v_oMo[0];
        }
        else {
            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
            oMo = v_oMo[1];   // Introduce PI rotation
        }

        
        
    } else if (cB_count == 2) {
        coor << "pd coordinate is " << std::endl;
        mask_detect.detect(Imot, opt_tagSize, cam, cdMo_vec);
        cdMo = cdMo_vec[cdMo_vec.size() - 1];
        std::vector<vpImagePoint> cd_corners = mask_detect.getPolygon(0);
        for (size_t i = 0; i < cd_corners.size(); i++) {
            vpFeatureBuilder::create(pd[i], cam, cd_corners[i]);
            vpColVector cP, p_;
            point[i].changeFrame(cdMo * oMo, cP);
            point[i].projection(cP, p_);
            pd[i].set_Z(cP[2]);
            coor << "x : " << pd[i].get_x() << "  y: " << pd[i].get_y() << " z : " << pd[i].get_Z() << std::endl;
        }
        coor << std::endl;
    }  else {
        first_time = false;
        mask_detect.detect(Imot, opt_tagSize, cam, cdMo_vec);
        cdMo = cdMo_vec[cdMo_vec.size() - 1];
        std::vector<vpImagePoint> cd_corners = mask_detect.getPolygon(0);
        coor << "pd coordinate is " << std::endl;
        for (size_t i = 0; i < cd_corners.size(); i++) {
            vpFeatureBuilder::create(pd[i], cam, cd_corners[i]);
            vpColVector cP, p_;
            point[i].changeFrame(cdMo * oMo, cP);
            point[i].projection(cP, p_);
            pd[i].set_Z(cP[2]);
            coor << "x : " << pd[i].get_x() << "  y: " << pd[i].get_y() << " z : " << pd[i].get_Z() << std::endl;
        }
        coor << std::endl;
    
        vpDisplay::display(Idisp);
        
        for (int i = 0; i < pd.size(); i++) {
            std::stringstream ss;
            vpImagePoint ip;
            ss << i;
            vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
            vpDisplay::displayText(Idisp, ip + vpImagePoint(15, 15), ss.str(), vpColor::green);
            vpDisplay::displayCross(Idisp, ip,  14, vpColor::green, 4);
        }
        for (int i = 0; i < p.size(); i++) {
            std::stringstream ss;
            vpImagePoint ip;
            ss << i;
            vpMeterPixelConversion::convertPoint(cam, p[i].get_x(), p[i].get_y(), ip);
            vpDisplay::displayText(Idisp, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
            vpDisplay::displayCross(Idisp, ip,  14, vpColor::red, 4);
        }
        detector.detect(Iin, opt_tagSize, cam, cMo_vec);
        vpColVector v_c(6);
        std::cout << "cMo_vec size is " << cMo_vec.size() << std::endl;
        cMo = cMo_vec[cMo_vec.size() - 1];
        std::vector<vpImagePoint> corners = detector.getPolygon(0);
        double pd_x = 0;
        double pd_y = 0;
        double pd_z = 0;
        //coor << "pd coordinate is " << std::endl;
        for (size_t i = 0; i < corners.size(); i++) {
            //coor << "x : " << pd[i].get_x() << "  y: " << pd[i].get_y() << " z : " << pd[i].get_Z() << std::endl;
            pd_x += pd[i].get_x();
            pd_y += pd[i].get_y();
            pd_z += pd[i].get_Z();
                
        }
        coor << "p coordinate is " << std::endl;
        for (size_t i = 0; i < corners.size(); i++) {
            vpFeatureBuilder::create(p[i], cam, corners[i]);
            vpColVector cP;
            point[i].changeFrame(cMo * oMo, cP);
            p[i].set_Z(cP[2]);
            
            coor << "x : " << p[i].get_x() << "  y: " << p[i].get_y() << " z : " << p[i].get_Z() << std::endl;
        }
        error << cB_count << " :  error is " << task.getError().sum() << std::endl;
        coor << std::endl;
        if (first_time) {
            first_time = false;
        }
        for (size_t j = 0; j < p.size(); j++) {
            std::ostringstream number;
            number << j;
        }
        vpDisplay::flush(Idisp);
    
        v = task.computeControlLaw();
        std::vector<double> bai;
        for (int i = 0; i < v.size(); i++) {
            bai.push_back(4);
        }
        double pd_m_z = 0;
        double p_m = 0;
        double pd_m_x = 0;
        double pd_m_y = 0;
        double p_m_x = 0;
        double p_m_y = 0;
        for (int i = 0; i < p.size(); i++) {
            pd_m_z += pd[i].get_Z();
            p_m += p[i].get_Z();
            pd_m_x += pd[i].get_x();
            pd_m_y += pd[i].get_y();
            p_m_x += p[i].get_x();
            p_m_y += p[i].get_y();

        }
        pd_m_z = pd_m_z / p.size();
        p_m = p_m / p.size();
        pd_m_x = pd_m_x / p.size();
        pd_m_y = pd_m_y / p.size();
        p_m_x = p_m_x / p.size();
        p_m_y = p_m_y / p.size();
        double err_1 = abs(pd_m_z - p_m);
        std::vector<double> twist_out;
        
        
       
       if (abs(task.getError().sum()) > first_step)
       {
           for (int i =0; i < bai.size(); i++) {
               bai[i] = 6;
           }
           for (int i = 0; i < bai.size(); i++) {
               twist_out.push_back(v[i] * bai[i]);
           }
       } else if (abs(task.getError().sum()) > second_step) {
           for (int i = 0; i < bai.size(); i++) {
               bai[i] = 2;
           }
           for (int i = 0; i < bai.size(); i++) {
               twist_out.push_back(v[i] * bai[i]);
           }
       } else {
           for (int i = 0; i < bai.size(); i++) {
               bai[i] = 0;
           }
           for (int i = 0; i < bai.size(); i++) {
               twist_out.push_back(v[i] * bai[i]);
           }
       }
        
        
        /*twist_out.push_back(v[0] * bai[0]);
        twist_out.push_back(v[1] * bai[1]);
        twist_out.push_back(v[2] * bai);
        twist_out.push_back(v[3] * bai);
        twist_out.push_back(v[4] * bai);
        twist_out.push_back(v[5] * bai);*/
        double baibai = 2;
        out_cmd.twist.linear.x = twist_out[0];
        out_cmd.twist.linear.y = twist_out[1];
        out_cmd.twist.linear.z = twist_out[2];
        out_cmd.twist.angular.x = twist_out[3] * baibai;
        out_cmd.twist.angular.y = twist_out[4] * baibai;
        out_cmd.twist.angular.z = twist_out[5] * baibai;
        ofs << v << std::endl;
        ofs << std::endl;
        ofs << out_cmd.twist << std::endl;
        ofs << std::endl;
        twist_pub.publish(out_cmd);
        image_pub.publish(visp_bridge::toSensorMsgsImage(Idisp));

    }
    
    
}
