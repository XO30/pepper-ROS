#!/usr/bin/env python2

import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

current_position = [0.0, 0.0, 0.0, 0.0, 0.0]

def interpolate_points(start, end, num_points):
    # Interpolate between start and end positions
    intermediate_points = []
    for i in range(num_points):
        alpha = float(i) / (num_points - 1)
        intermediate_point = [start[j] + alpha * (end[j] - start[j]) for j in range(len(start))]
        intermediate_points.append(intermediate_point)
    return intermediate_points

def move_arm_up():
    global current_position
    arm_pub = rospy.Publisher('/pepper_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(1)  # Wait for the arm controller to initialize

    # Define the joint names and target positions
    joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
    target_positions = [0.2, 0.2, -0.3, 0.7, 3.0]

    # Create a JointTrajectory message
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names

    # Add intermediate waypoints for smoother motion
    num_intermediate_points = 50
    intermediate_positions = interpolate_points(current_position, target_positions, num_intermediate_points)

    # Initialize time_from_start to zero
    time_from_start = rospy.Duration(0.0)

    for position in intermediate_positions:
        point = JointTrajectoryPoint()
        point.positions = position
        point.time_from_start = time_from_start
        trajectory_msg.points.append(point)

        # Increment time_from_start for the next point
        time_from_start += rospy.Duration(2.0 / num_intermediate_points)

    # Publish the arm movement command
    arm_pub.publish(trajectory_msg)
    current_position = target_positions

def move_arm_down():
    global current_position
    arm_pub = rospy.Publisher('/pepper_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
    rospy.sleep(1)  # Wait for the arm controller to initialize

    # Define the joint names and target positions for moving the arm down
    joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
    target_positions = [1.4, 0.1, 0.1, 0.1, 1.6]

    # Create a JointTrajectory message
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names

    # Add intermediate waypoints for smoother motion
    num_intermediate_points = 10
    intermediate_positions = interpolate_points(current_position, target_positions, num_intermediate_points)

    # Initialize time_from_start to zero
    time_from_start = rospy.Duration(0.0)

    for position in intermediate_positions:
        point = JointTrajectoryPoint()
        point.positions = position
        point.time_from_start = time_from_start
        trajectory_msg.points.append(point)

        # Increment time_from_start for the next point
        time_from_start += rospy.Duration(2.0 / num_intermediate_points)

    # Publish the arm movement command to move the arm down
    arm_pub.publish(trajectory_msg)
    current_position = target_positions

def callback(data):
    if data.data:  # If the message is True, move the arm up
        move_arm_up()
    else:  # If the message is False, move the arm down
        move_arm_down()

def arm_control_node():
    rospy.init_node('pepper_arm_node', anonymous=False)
    rospy.Subscriber('pepper_arm', Bool, callback)
    rospy.spin()

if __name__ == '__main__':
    arm_control_node()

