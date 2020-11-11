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
/*static const std::string PLANNING_GROUP = "manipulator";
moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);*/
const std::string detectorName = "ORB";
const std::string extractorName = "ORB";
const std::string matcherName = "BruteForce-Hamming";
vpColVector v;
vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
double opt_tagSize = 0.05;
int opt_quad_decimate = 2;
vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
vpDetectorAprilTag detector(tagFamily);
std::vector<vpHomogeneousMatrix> cMo_vec;
vpServo task;
std::vector<vpFeaturePoint> p(4), pd(4); 
std::ofstream ofs("/home/tsuchida/output_file/vel_1.txt");
std::ofstream coor("/home/tsuchida/output_file/coordinate.txt");
std::ofstream counts("/home/tsuchida/output_file/count.txt");
std::ofstream error("/home/tsuchida/output_file/error.txt");
ros::Publisher twist_pub, image_pub;
vpCameraParameters cam;
unsigned int width = 640, height = 480;
vpHomogeneousMatrix cMo, oMo;
vpHomogeneousMatrix cdMo( vpTranslationVector(0, 0, opt_tagSize * 3), // 3 times tag with along camera z axis
                               vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );
void CB(const sensor_msgs::ImageConstPtr&);
void camera_CB(const sensor_msgs::CameraInfoConstPtr&);
static bool first_time = true;
std::vector<vpPoint> point(4);
int cB_count = 0;
int count = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_servo");
    ros::NodeHandle nh_;
    
    cam.initPersProjWithoutDistortion(476, 476, 400, 300);
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    vpAdaptiveGain lamda(1.5, -0.4, 30);
    
    vpImageIo::read(Imot, mask_image);
    Idisp.resize(Imot.getHeight(), Imot.getWidth());
    Idisp.insert(Imot, vpImagePoint(0, 0));
    vpDisplayOpenCV d(Idisp, 0, 0, "Matcheng keypoints whti ORB keypoint");
    vpDisplay::display(Idisp);
    vpDisplay::flush(Idisp);
     
    point[0].setWorldCoordinates(-opt_tagSize/2., -opt_tagSize/2., 0);
    point[1].setWorldCoordinates( opt_tagSize/2., -opt_tagSize/2., 0);
    point[2].setWorldCoordinates( opt_tagSize/2.,  opt_tagSize/2., 0);
    point[3].setWorldCoordinates(-opt_tagSize/2.,  opt_tagSize/2., 0);

    
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);
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
    


    
   /* for (int i = 0; i < v.size(); i++) {
        v[i] = 0;
    }*/
    

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
    //std::cout << cam << std::endl;
}

void CB(const sensor_msgs::ImageConstPtr& msg)
{
   
    //vpKeyPoint keypoint(detectorName, extractorName, matcherName, filterType);

    //keypoint.buildReference(Imot);

    
    geometry_msgs::TwistStamped out_cmd;
    out_cmd.header.stamp.sec = 1599662324;
    out_cmd.header.stamp.nsec = 832165956;
    Iin = visp_bridge::toVispImage(*msg);
    Idisp.insert(Iin, vpImagePoint(0, 0));
    ROS_INFO("CB_count is %i", cB_count++);
    counts << "CB_count is " << cB_count << std::endl;

    
    // Introduce security wrt tag positionning in order to avoid PI rotation
    
    if (cB_count == 1) {
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

    // Compute the desired position of the features from the desired pose
        //std::cout << "pd coordinate is " << std::endl;
        
        coor << "pd coordinate is " << std::endl;
        for (size_t i = 0; i < point.size(); i++) {
            vpColVector cP, p_;
            point[i].changeFrame(cdMo * oMo, cP);
            point[i].projection(cP, p_);
            /*point[i].track(cdMo);
            vpFeatureBuilder::create(pd[i], point[i]);*/
            pd[i].set_x(p_[0]);
            pd[i].set_y(p_[1]);
            pd[i].set_Z(cP[2]);
            
            //std::cout << "x : " << pd[i].get_x() << "  y: " << pd[i].get_y() << " z : " << pd[i].get_Z() << std::endl;
            coor << "x : " << pd[i].get_x() << "  y: " << pd[i].get_y() << " z : " << pd[i].get_Z() << std::endl;
        }
        //std::cout << std::endl;
        coor << std::endl;
    }
    first_time = false;
   
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
    coor << "pd coordinate is " << std::endl;
    for (size_t i = 0; i < corners.size(); i++) {
        coor << "x : " << pd[i].get_x() << "  y: " << pd[i].get_y() << " z : " << pd[i].get_Z() << std::endl;
        pd_x += pd[i].get_x();
        pd_y += pd[i].get_y();
        pd_z += pd[i].get_Z();
            
    }
    int n = corners.size();
    pd_x = pd_x / n;
    pd_y = pd_y / n;
    pd_z = pd_z / n;
    coor << std::endl;
    double p_x = 0;
    double p_y = 0;
    double p_z = 0;
    coor << "p coordinate is " << std::endl;
    for (size_t i = 0; i < corners.size(); i++) {
        vpFeatureBuilder::create(p[i], cam, corners[i]);
        vpColVector cP;
        point[i].changeFrame(cMo * oMo, cP);
        p[i].set_Z(cP[2]);
        
        coor << "x : " << p[i].get_x() << "  y: " << p[i].get_y() << " z : " << p[i].get_Z() << std::endl;
        p_x += p[i].get_x();
        p_y += p[i].get_y();
        p_z += p[i].get_Z();
    }
    p_x = p_x / n;
    p_y = p_y / n;
    p_z = p_z / n;
    double sa_x = (p_x - pd_x) * (p_x - pd_x);
    double sa_y = (p_y - pd_y) * (p_y - pd_y);
    double sa_z = (p_z = pd_z) * (p_z - pd_z);
    double err = std::sqrt(sa_x + sa_y + sa_z);
    error << cB_count << " :  error is " << task.getError().sumSquare() << std::endl;
    /*for (size_t i = 0; i < corners.size(); i++) {
        std::stringstream ss;
        ss << i;
        vpDisplay::displayText(Idisp, corners[i] + vpImagePoint(15, 15), ss.str(), vpColor::red);
    }*/
    coor << std::endl;
    if (first_time) {
        first_time = false;
    }
   /* for (size_t i = 0; i < detector.getNbObjects(); i++) {
        std::vector<vpImagePoint> p = detector.getPolygon(i);
        vpRect bbox = detector.getBBox(i);
        vpDisplay::displayRectangle(Idisp, bbox, vpColor::green);
       
        std::string message = detector.getMessage(i);
        std::size_t tag_id_pos = message.find("id: ");
        if (tag_id_pos != std::string::npos) {
            int tag_id = atoi(message.substr(tag_id_pos + 4).c_str());
            std::stringstream ss;
            ss.str("");
            ss << "Tag id: " << tag_id;
            vpDisplay::displayText(Idisp, (int)(bbox.getTop() - 10), (int)bbox.getLeft(), ss.str(), vpColor::red);
        }
    }*/
    
    for (size_t j = 0; j < p.size(); j++) {
        //vpDisplay::displayCross(Iin, p[j], 14, vpColor::red, 3);
        std::ostringstream number;
        number << j;
        //vpDisplay::displayText(Iin, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
    }
    //vpDisplay::displayText(Iin, 20, 20, "Click to display tag poses", vpColor::red);
   vpDisplay::flush(Idisp);
    
    //unsigned int npMatch = keypoint.matchPoint(Iin);
    
    //Idisp.insert(Iin, vpImagePoint(0, Imot.getWidth()));
   // vpImageDraw::drawLine(Idisp, vpImagePoint(0, Iin.getWidth()), vpImagePoint(Iin.getHeight(), Iin.getWidth()), vpColor::white, 2);
    /*vpDisplay::display(Idisp);
    vpDisplay::displayLine(Idisp, vpImagePoint(0, Iin.getWidth()), vpImagePoint(Iin.getHeight(), Iin.getWidth()), 
                            vpColor::white, 2);
    vpDisplay::flush(Idisp);*/
    //vpDisplayOpenCV d(Iin, 0, 0, "Matcheng keypoints whti ORB keypoint");
    //vpDisplay::display(Iin);
   // vpDisplay::flush(Iin);
    //vpImagePoint iPref, iPcur;
   //double Z = move_group.getCurrentPose().pose.position.z;
  //  printf("%l", Z);
    //double Z = 1.0;
    
    //for (unsigned int i = 0; i < 20; i++) {
        //keypoint.getMatchedPoints(i, iPref, iPcur);
        //pd[i].set_xyZ(iPref.get_i(), iPref.get_j(), Z);
        //vpDisplay::displayLine(Idisp, iPref, iPcur + vpImagePoint(0, Iin.getWidth()), vpColor::green);
      //  vpImageDraw::drawLine(Idisp, iPref, iPcur + vpImagePoint(0, Iin.getWidth()), vpColor::green, 1);
        //p[i].set_xyZ(iPcur.get_i(), iPcur.get_j(), Z);
        //task.addFeature(p[i], pd[i]);
    //}
    
    //vpDisplay::flush(Iin);
    //std::cout << std::endl;
    

//vpDisplay::flush(Idisp);
    
    
    v = task.computeControlLaw();
    std::vector<double> twist_out;
    double err_1 = task.getError().sumSquare();
    int bai;
    if (err_1 > 0.02) {
        bai = 4;
    } else if (err_1 > 0.004) {
        bai = 1;
    } else {
        bai = 0;
    }

    twist_out.push_back(v[0] * bai);
    twist_out.push_back(v[1] * bai);
    twist_out.push_back(v[2] * bai);
    twist_out.push_back(v[3] * bai);
    twist_out.push_back(v[4] * bai);
    twist_out.push_back(v[5] * bai);
    /*twist_out.push_back(v[3] * 0);
    twist_out.push_back(v[4] * 0);
    twist_out.push_back(v[5] * 0);*/
    /*for (int i = 0; i < twist_out.size(); i++) {
        if (twist_out[i] < 0.03 && twist_out[i] > -0.03) {
            twist_out[i] = 0;
        } else if (twist_out[i] < -0.7 || twist_out[i] > 0.7) {
            twist_out[i] = 0;
        }
    }*/
   /* for (int i = 0; i < twist_out.size(); i++) {
        if (twist_out[i] < 0.03 && twist_out[i] >= 0) {
            twist_out[i] = 0.03;
        } else if (twist_out[i] < 0 && twist_out[i] > -0.03) {
            twist_out[i] = 0.03;
        } else if (twist_out[i] < -0.7) {
            twist_out[i] = -0.7;
        } else if (twist_out[i] > 0.7) {
            twist_out[i] = 0.7;
        }
    }*/


    out_cmd.twist.linear.x = twist_out[0];
    out_cmd.twist.linear.y = twist_out[1];
    out_cmd.twist.linear.z = twist_out[2];
    /*out_cmd.twist.linear.x = v[0] * 1000;
    out_cmd.twist.linear.y = v[1] * 1000;
    out_cmd.twist.linear.z = v[2]*8;*/
    
    out_cmd.twist.angular.x = twist_out[3];
    out_cmd.twist.angular.y = twist_out[4];
    out_cmd.twist.angular.z = twist_out[5];
    /*out_cmd.twist.angular.x = 0;
    out_cmd.twist.angular.y = 0;
    out_cmd.twist.angular.z = 0;*/
    ofs << v << std::endl;
    ofs << std::endl;
    ofs << out_cmd.twist << std::endl;
    ofs << std::endl;
    twist_pub.publish(out_cmd);
    
    
    
    //vpDisplay::getClick(Iin);
   /* vpDisplay::display(Idisp);
    for (size_t i = 0; i < cMo_vec.size(); i++) {
    vpDisplay::displayFrame(Idisp, cMo_vec[i], cam, opt_tagSize / 2, vpColor::none, 3);
    }
    //vpDisplay::displayText(Iin, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::flush(Idisp);*/
    //vpDisplay::getClick(Iin);
    
    image_pub.publish(visp_bridge::toSensorMsgsImage(Idisp));

    
}
