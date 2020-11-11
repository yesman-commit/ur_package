#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <sensor_msgs/CameraInfo.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDot.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPioneer.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <vector>

#include <cmath>
#include <iostream>
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /*get jacobian*/
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  sensor_msgs::JointState jointst;
  jointst.name = joint_model_group->getVariableNames();

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");
  


  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
  
  

  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

  /*Jacobian*/

  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  //ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  ros::NodeHandle n, nh, np;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  Eigen::MatrixXd in_jacobian = jacobian.inverse();
  double x = end_effector_state.translation().x();
  double y = end_effector_state.translation().y();
  double th = 0;
  
  geometry_msgs::Quaternion on = tf::createQuaternionMsgFromYaw(th);
  ROS_INFO("%f %f", x, y);

  double vx = 0.1;
  double vy = 0;
  double vth = 0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Publisher arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint points_n;
  traj.header.frame_id = "base_link";
  traj.joint_names.resize(6);
  traj.points.resize(6);
  vector<double> lx;
  lx.push_back(0.1);
  lx.push_back(0);
  lx.push_back(0.1);
  end_effector_state.translation().addTo(lx);
  ROS_INFO_STREAM("Translat" << end_effector_state.translation() << "\n");
  ros::Rate r(1.0);
  while (n.ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy*sin(th)) * dt;
    double delta_y = (vx * sin(th) - vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
    
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.header.stamp = current_time;
    odom_trans.child_frame_id = "base_link";

    
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //ROS_INFO("%f %f %f", x, y, th);

  }
  
  
  

  ros::shutdown();
  return 0;
}
