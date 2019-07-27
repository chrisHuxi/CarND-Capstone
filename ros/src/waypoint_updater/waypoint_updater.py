#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import Int32
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

LOOKAHEAD_WPS = 75 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.waypoints_2d = None #因为考虑到要先初始化再call back所以设一个 variable of self
        self.waypoint_tree = None
        self.pose = None
        
        self.stop_line_waypoint_index = -1
        

        
        self.loop()
        
        
        #rospy.spin()

    def loop(self):
        # TODO
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints: #如果两者都不是None
                self.publish_waypoints()
            rate.sleep()
        # TODO: END
    
    def get_closest_points_index(self):
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
    
    def publish_waypoints(self):
        # TODO
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        # TODO: END
        '''
        publish_msg = Lane()
        publish_msg.header = self.base_waypoints.header
        
        self.final_waypoints_pub.publish(publish_msg)
        '''
    def generate_lane(self):
        # TODO
        final_lane = Lane()

        closest_waypoint_index = self.get_closest_points_index()
        #rospy.logwarn("closest_waypoint_index:{0}".format(closest_waypoint_index))
        farthest_waypoint_index = closest_waypoint_index + LOOKAHEAD_WPS
        base_waypoints_slice = self.base_waypoints.waypoints[closest_waypoint_index:closest_waypoint_index+LOOKAHEAD_WPS]
        
        if self.stop_line_waypoint_index == -1 or self.stop_line_waypoint_index >= farthest_waypoint_index:
            final_lane.waypoints = base_waypoints_slice
        else:
            #final_lane.waypoints = base_waypoints_slice
            final_lane.waypoints = self.get_decelerate_waypoint(base_waypoints_slice, closest_waypoint_index)
        
        
        return final_lane
        # TODO: END
        
    def get_decelerate_waypoint(self, base_waypoints_slice, closest_waypoint_index):
        # TODO
        temp_waypoints_list = []
        #initial_velocity = base_waypoints_slice[0].twist.twist.linear.x
        
        #initial_distance_to_stop = self.distance(base_waypoints_slice, 0, stop_index)
        decelaration_in_theory = MAX_DECEL#(initial_velocity)*(initial_velocity)/(2.0*initial_distance_to_stop)
        
        for i,wp in enumerate(base_waypoints_slice):
            temp_waypoint = Waypoint()
            temp_waypoint.pose = wp.pose
            # we should stop a little bit in front of stop line to make sure safty
            stop_index = max(self.stop_line_waypoint_index - closest_waypoint_index - 2, 0)
            #rospy.logwarn("stop_index:{0}".format(stop_index))
            distance_to_stop = self.distance(base_waypoints_slice, i, stop_index)
            desire_velocity = math.sqrt(2 * decelaration_in_theory * distance_to_stop)
            # uniformly accelerated motion: v^2 = 2 * a * x
            if desire_velocity < 1.0:
                desire_velocity = 0.0
            temp_waypoint.twist.twist.linear.x = min(desire_velocity, temp_waypoint.twist.twist.linear.x)
            temp_waypoints_list.append(temp_waypoint)
        return temp_waypoints_list
    
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        # TODO: END
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints] 
            self.waypoint_tree = KDTree(self.waypoints_2d)
        # TODO: END
        
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_line_waypoint_index = msg.data
        #rospy.logwarn("stop_line_waypoint_index:{0}".format(self.stop_line_waypoint_index))
        # pass

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
