#!/usr/bin/env python2

import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random

state = False

def generate_talk_gesture_left():
    left_arm_pub = rospy.Publisher('/pepper_dcm/LeftArm_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(1)
    
    left_joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
    left_gesture_positions = [
        [0.3, 0.2, -0.3, 1.2, -2.7],
        [0.3,  -0.2, 0.3, -1.2, -2.7],
        [0.3, 0.2, -0.3, 1.2, -2.7],
        [0.3, -0.2, 0.3, -1.2, -2.7],
        [0.3, 0.0, 0.0, 0.0, -2.7],  # Neutral position (all joints at 0)
        [0.3, 0.5, -0.3, 0.7, -1.0],  # Custom position 1
        [0.3, -0.2, 0.2, -0.5, -2.0],  # Custom position 2
        [0.3, 0.3, -0.1, 0.3, -1.5],  # Custom position 3
        [0.3, 0.0, 0.0, 0.0, -2.7],  # Back to neutral position
        [0.5, 0.2, -0.3, -1.2, -2.7],  # Repeat initial position
        [0.3, 0.5, -0.3, 0.7, -1.0],  # Custom position 1
        [0.1, -0.2, 0.2, -0.5, -2.0],  # Custom position 2
        [0.1, 0.3, -0.1, 0.3, -1.5],  # Custom position 3
        [0.0, 0.0, 0.0, 0.0, -2.7],  # Back to neutral position
        [0.5, 0.2, -0.3, -1.2, -2.7] 
    ]

    trajectory_msg_left = JointTrajectory()
    trajectory_msg_left.joint_names = left_joint_names

    time_from_start = rospy.Duration(0.0)
    num_intermediate_points = 10

    start_position_left = left_gesture_positions[0]
    end_position_left = random.choice(left_gesture_positions)
    
    for j in range(num_intermediate_points + 1):
        intermediate_position_left = [
            start_position_left[k] + (end_position_left[k] - start_position_left[k]) * (j / float(num_intermediate_points))
            for k in range(len(start_position_left))
        ]

        point_left = JointTrajectoryPoint()
        point_left.positions = intermediate_position_left
        point_left.time_from_start = time_from_start
        trajectory_msg_left.points.append(point_left)
        time_from_start += rospy.Duration(0.1)

    left_arm_pub.publish(trajectory_msg_left)
    start_position_left = end_position_left

def generate_stop_gesture_left():
    left_arm_pub = rospy.Publisher('/pepper_dcm/LeftArm_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(1)
    
    left_joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
    left_stop_positions = [1.4, 0.1, 0.1, 0.1, -1.6]
    
    left_trajectory_msg = JointTrajectory()
    left_trajectory_msg.joint_names = left_joint_names
    left_point = JointTrajectoryPoint()
    left_point.positions = left_stop_positions
    left_point.time_from_start = rospy.Duration(2.0)
    left_trajectory_msg.points.append(left_point)

    left_arm_pub.publish(left_trajectory_msg)

def handle_gesticulation():
    global state
    
    while not rospy.is_shutdown():
        if state:
            random_moves_left = random.randint(2, 7)
            random_pause_left = random.randint(8, 13)

            for _ in range(random_moves_left):
                if state:
                    generate_talk_gesture_left()
                    rospy.sleep(3)
                else:
                    pass
            
            generate_stop_gesture_left()
            if state:
                rospy.sleep(random_pause_left)
            else:
                pass

        else:
            rospy.sleep(1) 

def callback(data):
    global state
    state = data.data

def gestic_node():
    rospy.init_node('pepper_left_gestic_node', anonymous=False)
    rospy.Subscriber('pepper_left_gestic', Bool, callback)

    handle_gesticulation()

    rospy.spin()

if __name__ == '__main__':
    gestic_node()
