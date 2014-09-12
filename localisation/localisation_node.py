#!/usr/bin/env python
import numpy as np

import rospy
import tf

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from map.srv import PositionOfClosestObstacle
from robot.srv import SensorMeasurements
from scripts.particle_filter import ParticleFilter
from scripts.velocity import Velocity
from scripts.pose import Pose
from scripts.motion_model import MotionModel
from scripts.measurement_model import MeasurementModel
from scripts.filter_parameters import MotionModelNoiseParameters, MeasurementModelParameters, FilterParameters

class LocalisationNode(object):
    def __init__(self):
        self.number_of_particles = int(rospy.get_param('~number_of_particles', '100'))
        self.number_of_random_particles = int(rospy.get_param('~number_of_random_particles', '5'))
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.number_of_readings = int(rospy.get_param('~number_of_readings', '180'))
        self.neg_x_limit = float(rospy.get_param('~neg_x_limit', '-5'))
        self.x_limit = float(rospy.get_param('~x_limit', '5'))
        self.neg_y_limit = float(rospy.get_param('~neg_y_limit', '-5'))
        self.y_limit = float(rospy.get_param('~y_limit', '5'))
        self.scanner_min_angle = float(rospy.get_param('~scanner_min_angle', '-1.57'))
        self.scanner_max_angle = float(rospy.get_param('~scanner_max_angle', '1.57'))
        self.scanner_angle_increment = float(rospy.get_param('~scanner_angle_increment', '0.0174'))
        self.scanner_min_range = float(rospy.get_param('~scanner_min_range', '0.0'))
        self.scanner_max_range = float(rospy.get_param('~scanner_max_range', '5.0'))
        self.hit_sigma = float(rospy.get_param('~hit_sigma', '1.'))
        self.alpha1 = float(rospy.get_param('~alpha1', '0.0001'))
        self.alpha2 = float(rospy.get_param('~alpha2', '0.0001'))
        self.alpha3 = float(rospy.get_param('~alpha3', '0.0001'))
        self.alpha4 = float(rospy.get_param('~alpha4', '0.0001'))

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(10))

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), "odom", "map")

        rospy.Subscriber('cmd_vel', Twist, self.update_filter)
        self.most_likely_pose_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        self.particle_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=200)

        motion_model_params = MotionModelNoiseParameters(self.alpha1, self.alpha2, self.alpha3, self.alpha4)
        measurement_model_params = MeasurementModelParameters(self.scanner_min_range, self.scanner_max_range, self.hit_sigma)
        filter_params = FilterParameters(self.neg_x_limit, self.x_limit, self.neg_y_limit, self.y_limit, self.number_of_particles, self.number_of_random_particles)
        self.particle_filter = ParticleFilter(motion_model_params, measurement_model_params, filter_params, self.generate_measurements)

        while not rospy.is_shutdown():
            self.update_filter()
            most_likely_pose = self.particle_filter.get_most_likely_pose()
            self.publish_transform(most_likely_pose)
            self.publish_markers(most_likely_pose)

    def publish_transform(self, most_likely_pose):
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

        transf_map_odom = np.dot(transf_map_base, np.linalg.inv(transf_odom_base))
        translation = tf.transformations.translation_from_matrix(transf_map_odom)
        quaternion = tf.transformations.quaternion_from_matrix(transf_map_odom)

        self.tf_broadcaster.sendTransform(translation, quaternion, rospy.Time.now(), "odom", "map")

    def update_filter(self, velocity_msg=None):
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

        self.particle_filter.iterate_filter(velocity, measurements)

    def generate_measurements(self, pose, sensor_frame):
        ranges = list()

        frame = sensor_frame
        translation_matrix = tf.transformations.translation_matrix([pose.x, pose.y, 0])
        quaternion = tf.quaternion_from_euler(0, 0, pose.heading)
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

        angle = laser_euler_rotation[2] - self.scanner_min_angle
        for i in xrange(self.number_of_readings):
            direction_x = cos(angle)
            direction_y = sin(angle)

            rospy.wait_for_service('position_of_closest_obstacle')
            try:
                proxy = rospy.ServiceProxy('position_of_closest_obstacle', PositionOfClosestObstacle)
                result = proxy(laser_translation[0], laser_translation[1], direction_x, direction_y)
                if result.success == 'success':
                    distance = self.distance(laser_translation[0], laser_translation[1], result.position_x, result.position_y)
                    distance_with_noise = self.hit_sigma * np.random.randn() + distance
                    if distance_with_noise < self.scanner_max_range:
                        ranges.append(distance_with_noise)
                    else:
                        ranges.append(self.scanner_max_range)
                else:
                    ranges.append(self.scanner_max_range)
            except rospy.ServiceException, e:
                rospy.logerr('position_of_closest_obstacle service call failed')
                ranges.append(self.scanner_max_range)

            angle = angle - self.angle_increment

        return ranges

    def publish_markers(self, most_likely_pose):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, most_likely_pose.heading)

        most_likely_pose_marker = Marker()
        most_likely_pose_marker.header.frame_id = 'map'
        most_likely_pose_marker.header.stamp = rospy.Time.now()
        most_likely_pose_marker.type = most_likely_pose_marker.ARROW
        most_likely_pose_marker.pose.position.x = most_likely_pose.x
        most_likely_pose_marker.pose.position.y = most_likely_pose.y
        most_likely_pose_marker.pose.position.z = most_likely_pose.y
        most_likely_pose_marker.pose.orientation.x = quaternion[0]
        most_likely_pose_marker.pose.orientation.y = quaternion[1]
        most_likely_pose_marker.pose.orientation.z = quaternion[2]
        most_likely_pose_marker.pose.orientation.w = quaternion[3]
        most_likely_pose_marker.color.a = 1.
        most_likely_pose_marker.color.r = 0.
        most_likely_pose_marker.color.g = 1.
        most_likely_pose_marker.color.b = 0.
        most_likely_pose_marker.scale.x = 0.5
        most_likely_pose_marker.scale.y = 0.5
        most_likely_pose_marker.scale.z = 0.5

        self.most_likely_pose_publisher.publish(most_likely_pose_marker)

        particle_marker_array = MarkerArray()
        for i,particle in enumerate(self.particle_filter.particles):
            quaternion = tf.transformations.quaternion_from_euler(0, 0, particle.pose.heading)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.ARROW
            marker.pose.position.x = particle.pose.x
            marker.pose.position.y = particle.pose.y
            marker.pose.position.z = particle.pose.y
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker.color.a = 1.
            marker.color.r = 0.
            marker.color.g = 0.
            marker.color.b = 1.
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5

            particle_marker_array.markers.append(marker)

        self.particle_publisher.publish(particle_marker_array)

if __name__ == '__main__':
    rospy.init_node('localisation')
    try:
        LocalisationNode()
    except rospy.ROSInterruptException: pass
