#!/usr/bin/env python2

import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random

state = False

def generate_talk_gesture_right():
    right_arm_pub = rospy.Publisher('/pepper_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(1)

    # Define the joint names and target positions for the right arm gesticulation
    right_joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
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

    right_gesture_positions = [[x[0], x[1], -x[2], x[3], -x[4]] for x in left_gesture_positions]

    # Create a JointTrajectory message
    trajectory_msg_right = JointTrajectory()
    trajectory_msg_right.joint_names = right_joint_names

    # Initialize time_from_start to zero
    time_from_start = rospy.Duration(0.0)

    # Define the number of intermediate points between each key point
    num_intermediate_points = 10

    # Add points to the trajectory for each position in the gesticulation sequence
    start_position_right = right_gesture_positions[0]

    end_position_right = random.choice(right_gesture_positions)
            
    for j in range(num_intermediate_points + 1):
        intermediate_position_right = [
            start_position_right[k] + (end_position_right[k] - start_position_right[k]) * (j / float(num_intermediate_points))
            for k in range(len(start_position_right))
        ]

        point_right = JointTrajectoryPoint()
        point_right.positions = intermediate_position_right
        point_right.time_from_start = time_from_start
        trajectory_msg_right.points.append(point_right)

        time_from_start += rospy.Duration(0.1)

    right_arm_pub.publish(trajectory_msg_right)
    start_position_right = end_position_right

def generate_stop_gesture_right():
    right_arm_pub = rospy.Publisher('/pepper_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(1)
    
    # Define the joint names and target positions to stop the right arm
    right_joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
    right_stop_positions = [1.4, 0.1, 0.1, -0.1, 1.6]

    # Create a JointTrajectory message for the right hand
    right_trajectory_msg = JointTrajectory()
    right_trajectory_msg.joint_names = right_joint_names
    right_point = JointTrajectoryPoint()
    right_point.positions = right_stop_positions
    right_point.time_from_start = rospy.Duration(2.0)
    right_trajectory_msg.points.append(right_point)

    # Publish the command to stop the right arm
    right_arm_pub.publish(right_trajectory_msg)

def handle_gesticulation():
    global state
    
    while not rospy.is_shutdown():
        if state:
            random_moves_right = random.randint(2, 7)
            random_pasue_right = random.randint(8, 13)

            for _ in range(random_moves_right):
                if state:
                    generate_talk_gesture_right()
                    rospy.sleep(3)
                else:
                    pass
                
            generate_stop_gesture_right()
            if state:
                rospy.sleep(random_pasue_right)
            else:
                pass

        else:
            rospy.sleep(1)

def callback(data):
    global state
    state = data.data

def gestic_node():
    rospy.init_node('pepper_right_gestic_node', anonymous=False)
    rospy.Subscriber('pepper_right_gestic', Bool, callback)
    
    handle_gesticulation()

    rospy.spin()

if __name__ == '__main__':
    gestic_node()