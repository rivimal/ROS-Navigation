#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty, EmptyRequest
import time

# Global variables to store covariance values
cov_x = 0.0
cov_y = 0.0
cov_z = 0.0

def amcl_pose_callback(msg):
    """
    Callback function to get covariance from the /amcl_pose topic.
    """
    global cov_x, cov_y, cov_z
    covariance = msg.pose.covariance
    cov_x = covariance[0]  # Covariance in x
    cov_y = covariance[7]  # Covariance in y
    cov_z = covariance[35]  # Covariance in z

def disperse_particles():
    """
    Calls the /global_localization service to disperse particles.
    """
    rospy.wait_for_service('/global_localization')
    try:
        disperse_service = rospy.ServiceProxy('/global_localization', Empty)
        rospy.loginfo("Calling /global_localization service to disperse particles...")
        disperse_service(EmptyRequest())
        rospy.loginfo("Particles dispersed.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def move_in_square():
    """
    Moves the robot in a square trajectory.
    """
    rospy.loginfo("Moving robot in a square trajectory...")
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    move_cmd = Twist()
    turn_cmd = Twist()
    
    # Parameters for square movement
    move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
    turn_cmd.angular.z = 0.5  # Turn at 0.5 rad/s
    move_duration = 4.0  # Time to move in a straight line (4 seconds)
    turn_duration = 3.2  # Time to make a 90-degree turn (3.2 seconds)

    # Move in a square (4 sides)
    for i in range(4):
        # Move forward
        rospy.loginfo("Moving forward...")
        start_time = time.time()
        while time.time() - start_time < move_duration:
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()

        # Stop before turning
        cmd_vel_pub.publish(Twist())
        time.sleep(1.0)

        # Turn 90 degrees
        rospy.loginfo("Turning...")
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            cmd_vel_pub.publish(turn_cmd)
            rate.sleep()

        # Stop after turning
        cmd_vel_pub.publish(Twist())
        time.sleep(1.0)

    rospy.loginfo("Square movement completed.")

def check_covariance():
    """
    Checks the covariance of particles and returns the maximum of cov_x, cov_y, and cov_z.
    """
    rospy.loginfo("Checking particle covariance...")
    return max(cov_x, cov_y, cov_z)

def main():
    """
    Main function to execute the task.
    """
    rospy.init_node('square_move', anonymous=True)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Step 1: Disperse particles
        disperse_particles()
        rospy.sleep(1.0)  # Give some time for the particles to disperse

        # Step 2: Perform square movement
        move_in_square()

        # Step 3: Check covariance
        rospy.sleep(1.0)  # Allow time for AMCL to update covariance
        covariance = check_covariance()
        rospy.loginfo("Covariance: x=%.3f, y=%.3f, z=%.3f | Max: %.3f", cov_x, cov_y, cov_z, covariance)

        # Step 4: Check if covariance is below threshold
        if covariance < 0.65:
            rospy.loginfo("Localization successful! Covariance is below threshold.")
            break
        else:
            rospy.logwarn("Covariance too high (%.3f). Repeating process...", covariance)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
