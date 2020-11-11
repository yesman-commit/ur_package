#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

def talker():
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    #pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)
    pub = rospy.Publisher('/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
    rospy.sleep(0.5)
    
    tes = Float64MultiArray()
   
    #tes.data = [math.pi/2, -math.pi/6.0, -math.pi/2, -math.pi/3, -math.pi/2, 0.0]
    tes.data = [-math.pi/2, -2.8 * math.pi/6, -math.pi/3, -1*math.pi/1, math.pi/2, 0];
    
    pub.publish(tes)
    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
