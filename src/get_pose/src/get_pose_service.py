#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger, TriggerResponse

# Global variable to store the robot's pose
current_pose = None

# Callback function to update the robot's pose
def pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

# Service callback function
def handle_get_pose(req):
    global current_pose
    if current_pose:
        response = TriggerResponse()
        response.success = True
        response.message = (
            f"Position: x={current_pose.position.x}, y={current_pose.position.y}, z={current_pose.position.z}\n"
            f"Orientation: x={current_pose.orientation.x}, y={current_pose.orientation.y}, "
            f"z={current_pose.orientation.z}, w={current_pose.orientation.w}"
        )
        return response
    else:
        return TriggerResponse(success=False, message="Pose data not yet available.")

def main():
    # Initialize the node
    rospy.init_node("get_pose_service_node")

    # Subscribe to the /amcl_pose topic (or other pose topic)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)

    # Create the Service Server
    rospy.Service("get_pose_service", Trigger, handle_get_pose)
    rospy.loginfo("Service Server 'get_pose_service' is ready...")

    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()
