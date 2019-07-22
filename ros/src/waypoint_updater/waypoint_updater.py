#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from scipy.spatial import KDTree
import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.waypoints_2d = None #因为考虑到要先初始化再call back所以设一个 variable of self
        self.waypoint_tree = None
        self.pose = None
        
        #rospy.spin()

    def loop(self):
        # TODO
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints: #如果两者都不是None
                closest_waypoint_index = self.get_closest_points_index()
                self.publish_waypoints(closest_waypoint_index)
            rate.sleep()
        # TODO: END
    
    def get_closest_points_index():
        # TODO
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        closest_index = self.waypoint_tree.query([x,y], 1)[1]
        closest_coord = self.waypoints_2d[closest_index]
        previous_coord = self.waypoints_2d[closest_index-1] #这里有疑问
        #上一个点：位置上，应该在后面
        
        #下一步判断coord在pose前还是后: 根据向量乘积的 negative和 positive 来判断
        closest_coord_vector = np.array(closest_coord)
        previous_coord_vector = np.array(previous_coord)
        pose_coord_vector = np.array([x,y])
        
        difference_vector1 = previous_coord_vector - pose_coord_vector
        difference_vector2 = closest_coord_vector - pose_coord_vector
        value = np.dot(difference_vector1,difference_vector2)
        
        if value > 0: #说明车在点前
            closest_index = (closest_index+1) % len(self.waypoints_2d)
            #选择下一个点（位置上，为前面那个点），因为离车最近的点在车子后，则下一个点一定在车前了
            #如果超出了界限，则从头开始，位置上应该是连续的（因为在simulator中是转圈的）
            
        return closest_index
        # TODO END
    
    def publish_waypoints(self,closest_index):
        publish_msg = Lane()
        publish_msg.header = self.base_waypoints.header
        publish_msg.waypoints = self.base_waypoints.waypoints[closest_index:closest_index+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(publish_msg)
    
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        # TODO: END
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoint
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints] 
            self.waypoint_tree = KDTree(self.waypoints_2d)
        # TODO: END
        
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
