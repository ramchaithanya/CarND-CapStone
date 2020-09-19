#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32
from scipy import interpolate

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

# Set Global Variables to use

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS_FOR_STOPPING = 50
MAX_SPEED = 11.111 # A predefined speed of 11.1111 m/s from waypoint to waypoint
STOPLINE_BUFFER = 2. # Car stops 2m infront of the stop line.
DISTANCE_TOO_CLOSE_FOR_STOPPING = 10.# If the car is at full speed and less then this distance ahead of the stop line


class WaypointUpdater(object):
    def __init__(self):

        self.waypoint_tree = None
        self.pose = None
        self.latest_stop_line_idx = -1
        self.stopping_start_position_found = False
        self.total_stop_lenght = 0
        self.curr_vel = None

        # Intiate the Ros Node
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.car_curr_vel_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.sub_tl_idx = rospy.Subscriber('/traffic_waypoints', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # Use spline for the velocity deceleration
        x = np.array([-0.1, 0., .1, .4, 1., 1.1])
        y = np.array([0., 0., .1, .5, 1., 1.])
        self.vel_spline = interpolate.splrep(x, y, s=0)

        self.loop_and_publishing()

        #rospy.spin()

    def loop_and_publishing(self):
        """
        This function generates waypoints based on the current position and base trajectory. It then publishes the planned trajectory waypoint.

        The loop cycle is set to 30Hz since the information is not updated frequently.
        """
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # ENDLESS LOOP
            if self.waypoint_tree and self.pose:
                wp_idx = self.get_closest_waypoint_idx()
                """
                wp_idx gets the waypoint of our current position.
                We then publish the coming 50 trajectory points based on
                the vehicles current waypoint ahead i.e wp_idx
                """
                self.publish_waypoints(wp_idx)
            # END OF LOOP
            rate.sleep()
        pass

    def get_closest_waypoint_idx(self):

        """
        This function gets the nearest waypoint index. It uses a KDTree Query data structure to get the current clossest waypoint.

        It then checks for the previous, current, and infront of the car position.

        it then returns the nearest index point.

        """
        closest_idx = self.waypoint_tree.query([self.pose.pose.position.x,
        self.pose.pose.position.y], 1)[1]

        # check for current, previous and clossest point to the vehicle
        closest_vec = np.array(self.waypoint_xy[closest_idx])
        prev_vec = np.array(self.waypoint_xy[closest_idx-1])
        curr_vec = np.array([self.pose.pose.position.x, self.pose.pose.position.y])
        val = np.dot(closest_vec - prev_vec, curr_vec - closest_vec)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoint_xy)
        return closest_idx

    def publish_waypoints(self, closest_idx):

        """
        This function performs the following:
        1. Publishes the LOOKAHEAD_WPS trajectory points of the base_waypoints to the final_waypoint topic.If the end of the base_waypoints are reached, the published waypoints gets shorter until there is
        no waypoint left.

        2. Publishes the trajectory based on whether it is required to stop or not. If there is a stop request from traffic_lights, then ==>(latest_stop_line_idx is not -1), then deceleration is planned.

        3. It then returns the final_waypoints_pub
        """

        lane = Lane()
        lane.header = self.all_waypoints.header
        base_waypoints = self.all_waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]

        if self.latest_stop_line_idx == -1 or (self.latest_stop_line_idx >= closest_idx + LOOKAHEAD_WPS_FOR_STOPPING):
            self.stopping_start_position_found = False
            lane.waypoints = base_waypoints
        else:
            if self.stopping_start_position_found == False:
                self.total_stop_lenght = self.distance(self.all_waypoints.waypoints, closest_idx, self.latest_stop_line_idx) - STOPLINE_BUFFER
                if not self.total_stop_lenght < DISTANCE_TOO_CLOSE_FOR_STOPPING:
                    self.stopping_start_position_found = True
            lane.waypoints = []

            for i, wp in enumerate(base_waypoints):
                curr_idx = closest_idx + i

                remaining_dist = self.distance(self.all_waypoints.waypoints, curr_idx, self.latest_stop_line_idx) - STOPLINE_BUFFER
                vel_at_dist = self.get_velocity(remaining_dist, self.total_stop_lenght, MAX_SPEED)
                p = Waypoint()
                p.pose = wp.pose
                p.twist.twist.linear.x = vel_at_dist
                lane.waypoints.append(p)
        self.final_waypoints_pub.publish(lane)

    def get_velocity(self, remaining_dist, total_dist, max_velocity):
        x_interp = np.array([min(remaining_dist / total_dist, 1.)])
        y_interp = interpolate.splev(x_interp, self.vel_spline, der=0)

        if y_interp[0] > 1.:
            y_interp[0] = 1.

        vel = abs(y_interp[0] * max_velocity)

        if vel < 0.2:
            vel = 0.
        return vel

    def pose_cb(self, msg):
        # TODO: Implement
        if not self.pose:
            rospy.loginfo("Recived first pose...")
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        """
        Callback for the base waypoints, which are provided once after startup. The waypoints contain pose and twist, however, we need the pose only. x and y cooridnates of the trajectory is maintained in a KDTree data stracture for efficiency(log(n)) and quicj access
        """
        self.all_waypoints = waypoints
        if not self.waypoint_xy:
            self.waypoint_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
#             for waypoint in waypoints.waypoints:
#                 self.waypoint_xy.append((waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            self.waypoint_tree = KDTree(self.waypoint_xy)
            rospy.loginfo("All waypoints Recived...")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.latest_stop_line_idx = msg.data

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
