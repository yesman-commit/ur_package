#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()

    group_names = robot.get_group_names()
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names




if __name__ == '__main__':
    rospy.init_node('move_group_python_inter', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
  
    planning_frame = group.get_planning_frame()
    current_pose = group.get_current_pose().pose
    print(current_pose)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.09
    pose_goal.position.y = 0.53
    pose_goal.position.z = 0.26
    pose_goal.orientation.x = -0.5
    pose_goal.orientation.y = -0.6
    pose_goal.orientation.z = 0.42
    pose_goal.orientation.w = 0.41
    group.set_goal_position_tolerance = 0.01
    group.set_goal_orientation_tolerance = 0.01
    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_target()
    current_pose = group.get_current_pose().pose
    if all_close(pose_goal, current_pose, 0.01):
        print("true")
    
    
    