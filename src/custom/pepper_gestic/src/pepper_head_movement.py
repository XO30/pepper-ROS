#!/usr/bin/env python2

import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random

state = False

def generate_random_head_movement():
    head_pub = rospy.Publisher('/pepper_dcm/Head_controller/command', JointTrajectory, queue_size=1)
    
    head_joint_names = ['HeadPitch', 'HeadYaw']
    head_ranges = {
        'HeadPitch': [-0.2, 0.2],
        'HeadYaw': [-0.6, 0.6]
    }

    trajectory_msg_head = JointTrajectory()
    trajectory_msg_head.joint_names = head_joint_names

    random_head_positions = [
        random.uniform(head_ranges['HeadPitch'][0], head_ranges['HeadPitch'][1]),
        random.uniform(head_ranges['HeadYaw'][0], head_ranges['HeadYaw'][1])
    ]

    point_head = JointTrajectoryPoint()
    point_head.positions = random_head_positions
    point_head.time_from_start = rospy.Duration(1.5)
    trajectory_msg_head.points.append(point_head)

    head_pub.publish(trajectory_msg_head)

def reset_head_position():
    head_pub = rospy.Publisher('/pepper_dcm/Head_controller/command', JointTrajectory, queue_size=1)
    
    head_joint_names = ['HeadPitch', 'HeadYaw']
    neutral_position = [0.0, 0.0]  # Neutralposition (alle Gelenke auf 0)

    trajectory_msg_head = JointTrajectory()
    trajectory_msg_head.joint_names = head_joint_names

    point_head = JointTrajectoryPoint()
    point_head.positions = neutral_position
    point_head.time_from_start = rospy.Duration(1.5)
    trajectory_msg_head.points.append(point_head)

    head_pub.publish(trajectory_msg_head)

def head_movement_loop():
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        if state:
            generate_random_head_movement()
        else:
            reset_head_position()
        rate.sleep()

def callback(data):
    global state
    state = data.data

def head_movement_node():
    rospy.init_node('pepper_random_head_movement', anonymous=False)
    rospy.Subscriber('pepper_head_movement_state', Bool, callback)
    head_movement_loop()
    rospy.spin()

if __name__ == '__main__':
    head_movement_node()
