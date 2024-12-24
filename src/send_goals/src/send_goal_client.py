#!/usr/bin/env python3

# Step a: Create a new package named send_goals with rospy as a dependency
# Command to create the package (run in terminal):
# catkin_create_pkg send_goals rospy actionlib move_base_msgs

# Step b: Write the Action Client code in send_goal_client.py


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def create_move_base_goal(x, y, z, qx, qy, qz, qw):
    """
    Create a MoveBaseGoal message.
    
    Args:
        x, y, z: Position coordinates.
        qx, qy, qz, qw: Quaternion for orientation.

    Returns:
        MoveBaseGoal: The goal message.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw
    return goal

def move_to_goals():
    """
    Send the robot to three predefined poses in a loop.
    """
    rospy.init_node('send_goal_client')

    # Define three poses
    poses = [
        (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        (2.5, -1.0, 0.0, 0.0, 0.0, 0.707, 0.707),
        (-1.0, 0.5, 0.0, 0.0, 0.0, -0.707, 0.707)
    ]

    # Create an action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server.")

    # Loop through the poses
    while not rospy.is_shutdown():
        for i, pose in enumerate(poses):
            goal = create_move_base_goal(*pose)
            rospy.loginfo(f"Sending goal {i + 1}: {pose}")
            client.send_goal(goal)

            # Wait for the robot to reach the goal
            client.wait_for_result()
            result = client.get_result()

            if result:
                rospy.loginfo(f"Goal {i + 1} reached successfully.")
            else:
                rospy.logwarn(f"Goal {i + 1} failed.")

if __name__ == '__main__':
    try:
        move_to_goals()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action client terminated.")
