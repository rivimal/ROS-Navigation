#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

def make_plan_client():
    rospy.init_node('make_plan_client')

    # Wait for the service to be available
    rospy.wait_for_service('/move_base/make_plan')

    try:
        # Create a service proxy to call the make_plan service
        make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        # Create a start pose (robot's current pose)
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"  # frame_id should be the same as the map frame
        start_pose.pose.position.x = 0.0  # Replace with your robot's starting position
        start_pose.pose.position.y = 0.0  # Replace with your robot's starting position
        start_pose.pose.orientation.w = 1.0  # No rotation

        # Create a goal pose (desired goal)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"  # frame_id should be the same as the map frame
        goal_pose.pose.position.x = 1.0  # Replace with your goal's position
        goal_pose.pose.position.y = 1.0  # Replace with your goal's position
        goal_pose.pose.orientation.w = 1.0  # No rotation

        # Call the make_plan service to get the plan from start to goal
        plan = make_plan(start_pose, goal_pose, 0.0)  # 0.0 is the tolerance

        # Print the plan (the path)
        rospy.loginfo("Received Plan:")
        for pose in plan.plan.poses:
            rospy.loginfo(pose)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    make_plan_client()
