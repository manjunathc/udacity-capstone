#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import tf

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
REFERENCE_VELOCITY = 11.0  	# 11.0 m/s = ~25mph

def get_closest_waypoint(x, y, yaw, waypoints):
	
	closest_pnt = -1
	closest_dist = 99999.9

	# search for the shortest distance between waypoint and current position
	for i in range(len(waypoints)):
		x_wp = waypoints[i-1].pose.pose.position.x
		y_wp = waypoints[i-1].pose.pose.position.y
		distance = ((x - x_wp)**2 + (y - y_wp)**2)**0.5
		if (distance < closest_dist):
			closest_dist = distance
			closest_pnt = i-1
			print(closest_pnt)
	
	# evaluation if waypoint is ahead or slightly behind the car
	x_closest = waypoints[closest_pnt].pose.pose.position.x
	y_closest = waypoints[closest_pnt].pose.pose.position.y

	# determine angle between position and closest waypoint 
	angle = np.arctan2((y_closest-y),(x_closest-y)) 

	# if behind the car, take the next point instead
	if (np.abs(yaw-angle) > np.pi/4): 		
		closest_pnt += 1
		# if new lap starts
		if (closest_pnt >= len(waypoints)):
			closest_pnt = 0
	
	return closest_pnt


class WaypointUpdater(object):
    def __init__(self):
	self.pose_updated = False	# did we receive an update of the current state yet
	self.pose_x = -1.0		# current x position 	
	self.pose_y = -1.0		# current y position
	self.pose_z = -1.0		# current z position
	self.roll = 0.0			# current roll
	self.pitch = 0.0		# current pitch
	self.yaw = 0.0			# current heading direction (yaw)
	self.current_velocity = 0.0	# current velocity of the car

	self.waypoints = []
	self.closest_waypoint = -1
	
        rospy.init_node('waypoint_updater')

        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

	self.sampling_rate = 50.0 # Rate for the main loop
        self.loop()
        #rospy.spin()

    def loop(self): 	# loop, that repeats with a rate of 'self.sammpling_ate'
	r = rospy.Rate(self.sampling_rate)       
        while not rospy.is_shutdown():
		if self.pose_updated:
			self.closest_waypoint = get_closest_waypoint(self.pose_x, self.pose_y, self.yaw, self.waypoints)
		if (self.closest_waypoint != -1):
			ahead = []
			n_waypoints = len(self.waypoints)
			if (n_waypoints > LOOKAHEAD_WPS):
				n_waypoints = LOOKAHEAD_WPS
			for i in range(n_waypoints):
				if (self.closest_waypoint + (i-1) < len(self.waypoints)):
					ahead.append(self.waypoints[self.closest_waypoint+(i-1)])
					self.set_waypoint_velocity(self.waypoints, self.closest_waypoint + (i-1), REFERENCE_VELOCITY)
				else:
					ahead.append(self.waypoints[self.closest_waypoint+i-len(self.waypoints)])
					self.set_waypoint_velocity(self.waypoints, self.closest_waypoint + (i-1) -len(self.waypoints), REFERENCE_VELOCITY)
			lane = Lane()
			lane.header.frame_id = '/world'
			lane.header.header.stamp = rospy.Time(0)
			lane.waypoints = ahead
			# publish the final waypoints
			self.final_waypoints_pub.publish(lane)
		r.sleep()

    def pose_cb(self, msg):
	'''
	rosmsg info geometry_msgs/PoseStamped:
 
	std_msgs/Header header
	  uint32 seq, time stamp, string frame_id
	geometry_msgs/Pose pose
	  geometry_msgs/Point position
	    float64 x, float64 y, float64 z
	  geometry_msgs/Quaternion orientation
	    float64 x, float64 y, float64 z, float64 w
	'''
	self.pose_x = msg.pose.position.x
	self.pose_y = msg.pose.position.y
	self.pose_z = msg.pose.position.z
	orientation = msg.pose.orientation
	# transforming the quaternion orientation into euler orientation
	orientation_euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
	# orientation_euler = [roll, pitch, yaw]
	self.roll = orientation_euler[0]
	self.pitch = orientation_euler[1]
	self.yaw = orientation_euler[2]
	self.pose_updated = True


    def waypoints_cb(self, waypoints):
	'''
	rosmsg info styx_msgs/Lane:

	std_msgs/Header header
	  uint32 seq, time stamp, string frame_id

	styx_msgs/Waypoint[] waypoints
	  geometry_msgs/PoseStamped pose
	    std_msgs/Header header
	      uint32 seq, time stamp, string frame_id
	    geometry_msgs/Pose pose
	      geometry_msgs/Point position
		float64 x, float64 y, float64 z,
	      geometry_msgs/Quaternion orientation
		float64 x, float64 y, float64 z, float64 w

	  geometry_msgs/TwistStamped twist
	    std_msgs/Header header
	      uint32 seq, time stamp, string frame_id
	    geometry_msgs/Twist twist
	      geometry_msgs/Vector3 linear
		float64 x, float64 y, float64 z
	      geometry_msgs/Vector3 angular
		float64 x, float64 y, float64 z
	'''
	# Saving the waypoints
        self.waypoints = waypoints.waypoints

	# Unsubscribe after waypoints are safed
	self.base_waypoints_sub.unregister()

    def current_velocity_cb(self, velocity):
	self.current_velocity = velocity.twist.linear.x

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
