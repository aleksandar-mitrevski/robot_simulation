from math import exp, sqrt, pi
from copy import copy
from pose import Pose

class MeasurementModel(object):
    '''Defines a utility for sampling a measurement model.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, model_params, measurement_generator_callback):
        self.model_params = copy(model_params)
        self.measurement_generator_callback = measurement_generator_callback

    def calculate_measurement_likelihood(self, pose, measurement_dict):
        '''Calculates the likelihood of the current sensor measurements for the given pose.

        Keyword arguments:
        pose - The pose of a particle.
        measurement_dict - A dictionary where each key is a name of a sensor frame and the values are measurements obtained with the respective sensor.
        '''
        likelihood = 1.
        max_range_measurements_counter = 0
        for sensor, measurements in measurement_dict.iteritems():
            pose_measurements = self.measurement_generator_callback(pose, sensor)
            for i,measurement in enumerate(measurements):
                if abs(measurement - self.model_params.max_range) > 0:
                    measurement_likelihood = self.likelihood(pose_measurements[i], measurement, self.model_params.hit_sigma)
                    likelihood = likelihood * measurement_likelihood
                else:
                    max_range_measurements_counter = max_range_measurements_counter + 1

        if max_range_measurements_counter > self.model_params.max_measurements_counter_tolerance:
            likelihood = 0.

        return likelihood

    def likelihood(self, x, mean, sigma):
        return 1. / (sigma * sqrt(2*pi)) * exp(-0.5 * (x - mean) * (x - mean)/(sigma * sigma))
