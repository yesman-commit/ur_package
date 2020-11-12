#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpImageSimulator.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <fstream>
void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot);
class vpVirtualGrabber
{
public:
  vpVirtualGrabber(const std::string &filename, const vpCameraParameters &cam) : sim_(), target_(), cam_()
  {
    // The target is a square 20cm by 2cm square
    // Initialise the 3D coordinates of the target corners
    for (int i = 0; i < 4; i++)
      X_[i].resize(3);
    // Top left      Top right        Bottom right    Bottom left
    X_[0][0] = -0.1;
    X_[1][0] = 0.1;
    X_[2][0] = 0.1;
    X_[3][0] = -0.1;
    X_[0][1] = -0.1;
    X_[1][1] = -0.1;
    X_[2][1] = 0.1;
    X_[3][1] = 0.1;
    X_[0][2] = 0;
    X_[1][2] = 0;
    X_[2][2] = 0;
    X_[3][2] = 0;
    vpImageIo::read(target_, filename);
    // Initialize the image simulator
    cam_ = cam;
    sim_.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
    sim_.init(target_, X_);
  }
  void acquire(vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
  {
    sim_.setCleanPreviousImage(true);
    sim_.setCameraPosition(cMo);
    sim_.getImage(I, cam_);
  }
private:
  vpColVector X_[4]; // 3D coordinates of the target corners
  vpImageSimulator sim_;
  vpImage<unsigned char> target_; // image of the target
  vpCameraParameters cam_;
};
void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot)
{
  static std::vector<vpImagePoint> traj[4];
  for (unsigned int i = 0; i < 4; i++) {
    traj[i].push_back(dot[i].getCog());
  }
  for (unsigned int i = 0; i < 4; i++) {
    for (unsigned int j = 1; j < traj[i].size(); j++) {
      vpDisplay::displayLine(I, traj[i][j - 1], traj[i][j], vpColor::green);
    }
  }
}
int main()
{
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
  try {
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    std::ofstream td("/home/tsuchida/program/velocity.txt");
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));
    vpImage<unsigned char> I(480, 640, 255);
    vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2);
    std::vector<vpPoint> point;
    point.push_back(vpPoint(-0.1, -0.1, 0));
    point.push_back(vpPoint(0.1, -0.1, 0));
    point.push_back(vpPoint(0.1, 0.1, 0));
    point.push_back(vpPoint(-0.1, 0.1, 0));
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    vpVirtualGrabber g("/home/tsuchida/program/visual_servo/ibvs/target_square.pgm", cam);
    g.acquire(I, cMo);
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 0, 0, "Current camera view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, 0, 0, "Current camera view");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I, 0, 0, "Current camera view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::display(I);
    vpDisplay::displayText(I, 10, 10, "Click in the 4 dots to initialise the tracking and start the servo",
                           vpColor::red);
    vpDisplay::flush(I);
    vpFeaturePoint p[4], pd[4];
    std::vector<vpDot2> dot(4);
    td << "point is ";
    for (unsigned int i = 0; i < 4; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      td << point[i] << " ";
      dot[i].setGraphics(true);
      dot[i].initTracking(I);
      vpDisplay::flush(I);
      vpFeatureBuilder::create(p[i], cam, dot[i].getCog());
      task.addFeature(p[i], pd[i]);
    }
    td << std::endl;
    vpHomogeneousMatrix wMc, wMo;
    vpSimulatorCamera robot;
    robot.setSamplingTime(0.040);
    robot.getPosition(wMc);
    wMo = wMc * cMo;
    td << "wMo is " << wMo << std::endl;
    td << "wMc is " << wMc << std::endl;
    td << "first cMo is " << cMo << std::endl;
    for (;;) {
      robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;
      td << "real time cMo is " << cMo << std::endl;
      g.acquire(I, cMo);
      vpDisplay::display(I);
      for (unsigned int i = 0; i < 4; i++) {
        dot[i].track(I);
        vpFeatureBuilder::create(p[i], cam, dot[i].getCog());
        vpColVector cP;
        point[i].changeFrame(cMo, cP);
        p[i].set_Z(cP[2]);
      }
      vpColVector v = task.computeControlLaw();
      td << "velocity is";
      for (int i = 0; i < v.size(); i++) {
          td  << v[i] << " , ";
      }
      td << "" << std::endl;
      display_trajectory(I, dot);
      vpServoDisplay::display(task, cam, I, vpColor::green, vpColor::red);
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      vpTime::wait(robot.getSamplingTime() * 1000);
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
