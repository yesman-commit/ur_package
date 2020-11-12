#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>
#include <ros/ros.h>
#include <visp_bridge/camera.h>
#include <visp3/gui/vpDisplayOpenCV.h>


void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint>  &vip, std::vector<vpImagePoint> *traj_vip)
{
    #if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
    for (size_t i = 0; i < vip.size(); i++) {
        if (traj_vip[i].size()) {
            if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1) {
                traj_vip[i].push_back(vip[i]);
            }
        } else
        {
            traj_vip[i].push_back(vip[i]);
        }
    }
    for (size_t i = 0; i < vip.size(); i++) {
        for (size_t j = 1; j < traj_vip[i].size(); j++) {
            vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
        }
    }
}

int main(int argc, char **argv)
{
    double opt_tagSize = 0.12;
    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(476, 476, 400, 300);
    vpImage<unsigned char> I;
    std::ofstream ofs("/home/tsuchida/output_file/vel_2.txt");
    std::ofstream coor("/home/tsuchida/output_file/coordinate_1.txt");
    bool display_tag = true;
    bool opt_verbose = false;
    int opt_quad_decimate = 2;
    vpImageIo::read(I, "/home/tsuchida/image/hit_2.jpg");
  //  vpDisplayOpenCV d(I, 0, 0, "Matcheng keypoints whti ORB keypoint");
  
    #ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#endif
    vpDisplay::display(I);
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h10;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);
    vpHomogeneousMatrix cdMc, cMo, oMo;
    vpHomogeneousMatrix cdMo( vpTranslationVector(0, 0, opt_tagSize * 3), // 3 times tag with along camera z axis
                               vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );
    std::vector<vpFeaturePoint> p(4), pd(4);
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-opt_tagSize/2., -opt_tagSize/2., 0);
    point[1].setWorldCoordinates( opt_tagSize/2., -opt_tagSize/2., 0);
    point[2].setWorldCoordinates( opt_tagSize/2.,  opt_tagSize/2., 0);
    point[3].setWorldCoordinates(-opt_tagSize/2.,  opt_tagSize/2., 0);
    vpServo task;
    for (size_t i = 0; i < p.size(); i++) {
        task.addFeature(p[i], pd[i]);
    }
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    bool first_time = true;
    std::vector<vpHomogeneousMatrix> cMo_vec;
    detector.detect(I, opt_tagSize, cam, cMo_vec);
    vpColVector v(6);
   
        cMo = cMo_vec[0];
        if (first_time) {
        // Introduce security wrt tag positionning in order to avoid PI rotation
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
            pd[i].set_x(p_[0]);
            pd[i].set_y(p_[1]);
            pd[i].set_Z(cP[2]);
            //std::cout << "x : " << pd[i].get_x() << "  y: " << pd[i].get_y() << " z : " << pd[i].get_Z() << std::endl;
            coor << "x : " << pd[i].get_x() << "  y: " << pd[i].get_y() << " z : " << pd[i].get_Z() << std::endl;
        }
        //std::cout << std::endl;
        coor << std::endl;

    }

// Get tag corners
    std::vector<vpImagePoint> corners = detector.getPolygon(0);

// Update visual features
    //std::cout << "p coordinate is " << std::endl;
    coor << "p coordinate is " << std::endl;
    for (size_t i = 0; i < corners.size(); i++) {
        // Update the point feature from the tag corners location
        vpFeatureBuilder::create(p[i], cam, corners[i]);
        // Set the feature Z coordinate from the pose
        vpColVector cP;
        point[i].changeFrame(cMo, cP);

        p[i].set_Z(cP[2]);
        //std::cout << "x : " << p[i].get_x() << "  y: " << p[i].get_y() << " z : " << p[i].get_Z() << std::endl;
        coor << "x : " << p[i].get_x() << "  y: " << p[i].get_y() << " z : " << p[i].get_Z() << std::endl;

    }
    //std::cout << std::endl;
    coor << std::endl;

//vpDisplay::flush(Idisp);
    v = task.computeControlLaw();
    std::vector<double> twist_out;
    twist_out.push_back(v[0] * 10);
    twist_out.push_back(v[1] * 50);
    twist_out.push_back(v[2]*10);
    twist_out.push_back(v[3]);
    twist_out.push_back(v[4] * 10);
    twist_out.push_back(v[5] * 10);
    /*for (int i = 0; i < twist_out.size(); i++) {
        if (twist_out[i] < 0.03 && twist_out[i] > -0.03) {
            twist_out[i] = 0;
        } else if (twist_out[i] < -0.7 || twist_out[i] > 0.7) {
            twist_out[i] = 0;
        }
    }*/
    for (int i = 0; i < twist_out.size(); i++) {
        if (twist_out[i] < 0.03 && twist_out[i] >= 0) {
            twist_out[i] = 0.03;
        } else if (twist_out[i] < 0 && twist_out[i] > -0.03) {
            twist_out[i] = 0.03;
        } else if (twist_out[i] < -0.7) {
            twist_out[i] = -0.7;
        } else if (twist_out[i] > 0.7) {
            twist_out[i] = 0.7;
        }
    }
    //vpServoDisplay::display(task, cam, I);
    for (size_t i = 0; i < corners.size(); i++) {
        std::stringstream ss;
        ss << i;
        vpDisplay::displayText(I, corners[i] + vpImagePoint(15, 15), ss.str(), vpColor::red);
        vpImagePoint ip;
        vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
        vpDisplay::displayText(I, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
        
    }
    vpDisplay::flush(I);
    vpDisplay::getClick(I);
    #else
  (void)argc;
  (void)argv;
  return 0;
    #endif
    
    
}