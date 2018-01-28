#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import sys

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            # print ("=-=-=-=-=-=-=-=-=-= elif =-=-=-=-=-=-=-=-=-=")
            # print ("light_wp1 ", light_wp)
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.UNKNOWN or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            # print ("light_wp2 ", light_wp)
            # print ("state ", state)
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            # self.upcoming_red_light_pub.publish(Int32(light_wp))
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))


    def get_closest_waypoint(self, pose_x, pose_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        ret_val = -1;
        if (self.waypoints is None or pose_x is None or pose_y is None):
            return ret_val

        min_dist = 1e9
        for idx, wp in enumerate(self.waypoints.waypoints):
            wp_x = wp.pose.pose.position.x
            wp_y = wp.pose.pose.position.y

            dist = self.get_euclidean_distance(wp_x, wp_y, pose_x, pose_y)
            if (dist < min_dist):
                min_dist = dist
                ret_val = idx
        return ret_val

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False
	self.camera_image.encoding = 'rgb8'
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        min_dist = 80;
        closest_light_idx = -1
        state = TrafficLight.UNKNOWN   # Default state
        stop_line_positions = self.config['stop_line_positions']
        
        if (self.waypoints is None):
            return closest_light_idx, state
        
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

        
        # Find closest stopping position to current pose
        closest_light_dist = sys.maxint
        
        for i in range(len(stop_line_positions)):
            p1 = stop_line_positions[i]
            p2 = self.pose.pose.position
            dist = self.get_euclidean_distance(p1[0], p1[1], p2.x, p2.y)
            if dist < closest_light_dist:
                closest_light_dist = dist
                closest_light_idx = i

        # rospy.loginfo('Closest Light Index: {},  at Distance={}'.format(closest_light_idx, closest_light_dist))

        if (closest_light_dist >= min_dist):
            return -1, state
        else:
            closest_light_waypoint_idx = self.get_closest_waypoint(stop_line_positions[closest_light_idx][0], stop_line_positions[closest_light_idx][1])
            state = self.get_light_state(stop_line_positions[closest_light_idx])
            # rospy.loginfo('Light state={}, light waypoint={}'.format(state, closest_light_waypoint_idx))

            return closest_light_waypoint_idx, state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
