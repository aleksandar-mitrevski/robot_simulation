#!/usr/bin/env python
import numpy as np
from math import cos, sin, sqrt
from threading import Lock

import rospy
import tf

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from map.srv import GetMap
from robot.srv import SensorMeasurements
from scripts.particle_filter import ParticleFilter
from scripts.velocity import Velocity
from scripts.pose import Pose
from scripts.coordinates import Coordinates
from scripts.motion_model import MotionModel
from scripts.measurement_model import MeasurementModel
from scripts.filter_parameters import MotionModelNoiseParameters, MeasurementModelParameters, FilterParameters

class LocalisationNode(object):
    '''Defines a node that takes care of localising a robot and updating its belief appropriately.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self):
        self.number_of_particles = int(rospy.get_param('~number_of_particles', '100'))

        #number of random particles that should be added at each resampling step
        self.number_of_random_particles = int(rospy.get_param('~number_of_random_particles', '5'))

        self.map_frame = rospy.get_param('~map_frame', '/map')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        #limits of the coordinates represented by a known map (in meters)
        self.neg_x_limit = float(rospy.get_param('~neg_x_limit', '-5'))
        self.x_limit = float(rospy.get_param('~x_limit', '5'))
        self.neg_y_limit = float(rospy.get_param('~neg_y_limit', '-5'))
        self.y_limit = float(rospy.get_param('~y_limit', '5'))

        self.scanner_min_angle = float(rospy.get_param('~scanner_min_angle', '-1.57'))
        self.scanner_max_angle = float(rospy.get_param('~scanner_max_angle', '1.57'))
        self.scanner_angle_increment = float(rospy.get_param('~scanner_angle_increment', '0.0174'))
        self.scanner_min_range = float(rospy.get_param('~scanner_min_range', '0.0'))
        self.scanner_max_range = float(rospy.get_param('~scanner_max_range', '5.0'))
        self.number_of_readings = int(rospy.get_param('~number_of_readings', '180'))
        self.hit_sigma = float(rospy.get_param('~hit_sigma', '0.5'))

        #used for discarding scans
        self.max_measurements_counter_tolerance = int(rospy.get_param('~max_measurements_counter_tolerance', '3'))

        #if the sum of the particle weights falls below this tolerance,
        #a completely new set of particles will be created
        self.weight_sum_tolerance = float(rospy.get_param('~weight_sum_tolerance', '0.000001'))

        #motion model parameters
        self.alpha1 = float(rospy.get_param('~alpha1', '0.00001'))
        self.alpha2 = float(rospy.get_param('~alpha2', '0.00001'))
        self.alpha3 = float(rospy.get_param('~alpha3', '0.00001'))
        self.alpha4 = float(rospy.get_param('~alpha4', '0.00001'))

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(10))

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), self.odom_frame, self.map_frame)
        self.filter_lock = Lock()

        rospy.Subscriber('cmd_vel', Twist, self.update_filter)
        self.most_likely_pose_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        self.particle_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=200)

        motion_model_params = MotionModelNoiseParameters(self.alpha1, self.alpha2, self.alpha3, self.alpha4)
        measurement_model_params = MeasurementModelParameters(self.scanner_min_range, self.scanner_max_range, self.hit_sigma, self.max_measurements_counter_tolerance)
        filter_params = FilterParameters(self.neg_x_limit, self.x_limit, self.neg_y_limit, self.y_limit, self.number_of_particles, self.number_of_random_particles, self.weight_sum_tolerance)
        self.particle_filter = ParticleFilter(motion_model_params, measurement_model_params, filter_params, self.generate_measurements)

        rospy.wait_for_service('get_map')
        try:
            proxy = rospy.ServiceProxy('get_map', GetMap)
            map_msg = proxy()
            self.map_columns = map_msg.occupancy_grid.width
            self.map_rows = map_msg.occupancy_grid.height
            self.map_resolution = map_msg.occupancy_grid.resolution
            self.map_width = self.map_columns * self.map_resolution
            self.map_height = self.map_rows * self.map_resolution
            self.map_grid = np.array(map_msg.occupancy_grid.data).reshape((self.map_rows, self.map_columns))
            self.map_x_boundaries = (-self.map_width/2., self.map_width/2.)
            self.map_y_boundaries = (-self.map_height/2., self.map_height/2.)
        except rospy.ServiceException:
            print 'get_map call failed'

        while not rospy.is_shutdown():
            self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), self.odom_frame, self.map_frame)
            self.update_filter()
            #most_likely_pose = self.particle_filter.get_most_likely_pose()
            #self.publish_transform(most_likely_pose)
            #self.publish_markers(most_likely_pose)

    def publish_transform(self, most_likely_pose):
        '''Publishes the current most likely pose.

        Keyword arguments:
        most_likely_pose -- A 'pose.Pose' object.

        '''
        translation_matrix = tf.transformations.translation_matrix([most_likely_pose.x, most_likely_pose.y, 0])
        quaternion = tf.transformations.quaternion_from_euler(0, 0, most_likely_pose.heading)
        quaternion_matrix = tf.transformations.quaternion_matrix(quaternion)
        transf_map_base = np.dot(translation_matrix, quaternion_matrix)

        try:
            (translation, quat_rotation) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        translation_matrix = tf.transformations.translation_matrix(translation)
        quaternion_matrix = tf.transformations.quaternion_matrix(quat_rotation)
        transf_odom_base = np.dot(translation_matrix, quaternion_matrix)

        #we want to publish the transform from map to odom; in order to do that,
        #we calculate the product of the transforms map->base and base->odom
        transf_map_odom = np.dot(transf_map_base, np.linalg.inv(transf_odom_base))
        translation = tf.transformations.translation_from_matrix(transf_map_odom)
        quaternion = tf.transformations.quaternion_from_matrix(transf_map_odom)

        self.tf_broadcaster.sendTransform(translation, quaternion, rospy.Time.now(), self.odom_frame, self.map_frame)

    def update_filter(self, velocity_msg=None):
        '''Calls the particle filter to update the localisation of the robot.
        If 'velocity_msg' is None, updates the filter with a zero motion command.
        '''
        velocity = Velocity()
        if velocity_msg != None:
            velocity.linear_x = velocity_msg.linear.x
            velocity_linear_y = velocity_msg.angular.z

        measurements = dict()
        rospy.wait_for_service('sensor_measurements')
        try:
            proxy = rospy.ServiceProxy('sensor_measurements', SensorMeasurements)
            result = proxy()
            if result.success:
                for _,scan_msg in enumerate(result.scans):
                    measurements[scan_msg.header.frame_id] = scan_msg.ranges
        except rospy.ServiceException, e:
            rospy.logerr('sensor_measurements service call failed')

        with self.filter_lock:
            self.particle_filter.iterate_filter(velocity, measurements)
            most_likely_pose = self.particle_filter.get_most_likely_pose()
            self.publish_markers(most_likely_pose)

    def generate_measurements(self, pose, sensor_frame):
        '''Calls a service from the 'map' package in order to find what the robot would sense if it was in the given pose.

        Keyword arguments:
        pose -- A 'Pose' object represernting the pose of a particle.
        sensor_frame -- The frame of the sensor that should generate the hypothetical measurements.

        '''
        ranges = list()

        frame = sensor_frame
        translation_matrix = tf.transformations.translation_matrix([pose.x, pose.y, 0])
        quaternion = tf.transformations.quaternion_from_euler(0, 0, pose.heading)
        quaternion_matrix = tf.transformations.quaternion_matrix(quaternion)
        transf_odom_base = np.dot(translation_matrix, quaternion_matrix)

        self.tf_listener.waitForTransform(self.base_frame, frame, rospy.Time(0), rospy.Duration(10))
        try:
            (translation, quat_rotation) = self.tf_listener.lookupTransform(self.base_frame, frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return ranges

        laser_translation_matrix = tf.transformations.translation_matrix(translation)
        laser_quaternion_matrix = tf.transformations.quaternion_matrix(quat_rotation)
        transf_base_laser = np.dot(laser_translation_matrix, laser_quaternion_matrix)

        transf_odom_laser = np.dot(transf_odom_base, transf_base_laser)
        laser_translation = tf.transformations.translation_from_matrix(transf_odom_laser)
        laser_euler_rotation = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_from_matrix(transf_odom_laser))

        directions_x = list()
        directions_y = list()
        angle = laser_euler_rotation[2] - self.scanner_min_angle
        for i in xrange(self.number_of_readings):
            direction_x = cos(self.normalise_angle(angle))
            directions_x.append(direction_x)
            direction_y = sin(self.normalise_angle(angle))
            directions_y.append(direction_y)
            angle = angle - self.scanner_angle_increment

        for i in xrange(self.number_of_readings):
            position = Coordinates(laser_translation[0], laser_translation[1])
            direction = Coordinates(directions_x[i], directions_y[i])
            obstacle_position, position_inside_map = self.find_closest_obstacle(position, direction)
            if position_inside_map:
                distance = self.distance(laser_translation[0], laser_translation[1], obstacle_position.x, obstacle_position.y)
                distance_with_noise = self.hit_sigma * np.random.randn() + distance
                if distance_with_noise < self.scanner_max_range:
                    ranges.append(distance_with_noise)
                else:
                    ranges.append(self.scanner_max_range)
            else:
                ranges.append(self.scanner_max_range)

        return ranges

    def publish_markers(self, most_likely_pose):
        '''Publishes the particles and the most likely pose for visualisation.
        '''
        quaternion = tf.transformations.quaternion_from_euler(0, 0, most_likely_pose.heading)

        most_likely_pose_marker = Marker()
        most_likely_pose_marker.header.frame_id = self.map_frame
        most_likely_pose_marker.header.stamp = rospy.Time.now()
        most_likely_pose_marker.type = most_likely_pose_marker.ARROW
        most_likely_pose_marker.pose.position.x = most_likely_pose.x
        most_likely_pose_marker.pose.position.y = most_likely_pose.y
        most_likely_pose_marker.pose.position.z = 0
        most_likely_pose_marker.pose.orientation.x = quaternion[0]
        most_likely_pose_marker.pose.orientation.y = quaternion[1]
        most_likely_pose_marker.pose.orientation.z = quaternion[2]
        most_likely_pose_marker.pose.orientation.w = quaternion[3]
        most_likely_pose_marker.color.a = 1.
        most_likely_pose_marker.color.r = 0.
        most_likely_pose_marker.color.g = 1.
        most_likely_pose_marker.color.b = 0.
        most_likely_pose_marker.scale.x = 0.5
        most_likely_pose_marker.scale.y = 0.2
        most_likely_pose_marker.scale.z = 0.1

        self.most_likely_pose_publisher.publish(most_likely_pose_marker)

        #~ particle_marker_array = MarkerArray()
        #~ for i,particle in enumerate(self.particle_filter.particles):
            #~ quaternion = tf.transformations.quaternion_from_euler(0, 0, particle.pose.heading)
#~ 
            #~ marker = Marker()
            #~ marker.header.frame_id = self.base_frame
            #~ marker.header.stamp = rospy.Time.now()
            #~ marker.type = marker.ARROW
            #~ marker.pose.position.x = particle.pose.x
            #~ marker.pose.position.y = particle.pose.y
            #~ marker.pose.position.z = 0
            #~ marker.pose.orientation.x = quaternion[0]
            #~ marker.pose.orientation.y = quaternion[1]
            #~ marker.pose.orientation.z = quaternion[2]
            #~ marker.pose.orientation.w = quaternion[3]
            #~ marker.color.a = 1.
            #~ marker.color.r = 0.
            #~ marker.color.g = 0.
            #~ marker.color.b = 1.
            #~ marker.scale.x = 0.5
            #~ marker.scale.y = 0.2
            #~ marker.scale.z = 0.1
#~ 
            #~ particle_marker_array.markers.append(marker)
#~ 
        #~ self.particle_publisher.publish(particle_marker_array)

    def find_closest_obstacle(self, position, direction):
        '''Finds the obstacle closest to the point at 'position'
        and in direction given by the direction vector 'direction'.

        Keyword arguments:
        position -- A 'pose.Pose' object.
        direction -- A 'pose.Pose' object.

        '''
        t = 0.
        t_increment = self.map_resolution
        point_position = Coordinates(position.x, position.y)
        position_inside_map = True
        obstacle_position = Coordinates(self.map_width, self.map_height)

        while position_inside_map:
            try:
                map_coordinates = self.world_to_map_coordinates(point_position.x, point_position.y)
                if self.map_grid[map_coordinates.x, map_coordinates.y] > 95:
                    obstacle_position = point_position
                    break
                t = t + t_increment
                point_position = position + direction.multiply(t)
            except ValueError:
                position_inside_map = False

        return obstacle_position, position_inside_map

    def world_to_map_coordinates(self, x, y):
        '''Converts the world coordinates (x,y) to map coordinates.
        '''
        if self.invalid_coordinates(x, y):
            raise ValueError('OccupancyGridMap: Invalid coordinates')
        column = int((x - self.map_x_boundaries[0]) / self.map_resolution)
        row = int((y - self.map_y_boundaries[0]) / self.map_resolution)
        return Coordinates(row, column)

    def invalid_coordinates(self, x, y):
        '''Returns True if the given coordinates fall outside the map and False otherwise.
        '''
        return x < self.map_x_boundaries[0] or x > self.map_x_boundaries[1] or y < self.map_y_boundaries[0] or y > self.map_y_boundaries[1]

    def distance(self, point1_x, point1_y, point2_x, point2_y):
        '''Calculates the distance between a point with coordinates
        (point1_x, point1_y) and a point with coordinates (point2_x, point2_y).
        '''
        return sqrt((point1_x - point2_x) * (point1_x - point2_x) + (point1_y - point2_y) * (point1_y - point2_y))

    def normalise_angle(self, angle):
        '''Converts 'angle' to the range [0,2*pi).
        '''
        if angle < 0.:
            angle = angle + 2 * 3.14
        return angle

if __name__ == '__main__':
    rospy.init_node('localisation')
    try:
        LocalisationNode()
    except rospy.ROSInterruptException: pass
