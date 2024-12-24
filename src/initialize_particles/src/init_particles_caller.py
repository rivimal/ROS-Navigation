#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest

def call_global_localization():
    # Initialize ROS node
    rospy.init_node('init_particles_client', anonymous=True)
    rospy.loginfo("Waiting for the /global_localization service...")

    # Wait for the service to become available
    rospy.wait_for_service('/global_localization')

    try:
        # Create a service client for the /global_localization service
        global_localization = rospy.ServiceProxy('/global_localization', Empty)

        # Send a request to disperse the particles
        rospy.loginfo("Calling /global_localization service to disperse particles...")
        response = global_localization(EmptyRequest())

        rospy.loginfo("Particles have been initialized successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    try:
        call_global_localization()
    except rospy.ROSInterruptException:
        pass
