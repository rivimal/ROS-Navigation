#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetMap

def call_static_map_service():
    rospy.wait_for_service('/static_map')
    try:
        static_map_service = rospy.ServiceProxy('/static_map', GetMap)
        response = static_map_service()
        
        map_data = response.map
        resolution = map_data.info.resolution
        width = map_data.info.width
        height = map_data.info.height

        rospy.loginfo(f"Map resolution: {resolution} meters/pixel")
        rospy.loginfo(f"Map dimensions: {width} x {height} pixels")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('call_map_service_client', anonymous=True)
    call_static_map_service()
