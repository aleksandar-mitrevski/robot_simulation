from math import exp, sqrt, pi
from copy import copy
from pose import Pose

class MeasurementModel(object):
    def __init__(self, model_params, measurement_generator_callback):
        self.model_params = copy(model_params)
        self.measurement_generator_callback = measurement_generator_callback

    def calculate_measurement_likelihood(self, pose, measurement_dict):
        '''
        Keyword arguments:
        pose - The pose of a particle.
        measurement_dict - A dictionary where each key is a name of sensor frames 
                       and the values are measurements obtained with the respective sensor.
        '''
        likelihood = 1.
        for sensor, measurements in measurement_dict.iteritems():
            pose_measurements = self.measurement_generator_callback(pose, sensor)
            for i,measurement in enumerate(measurements):
                if abs(measurement - self.model_params.max_range) > 0:
                    measurement_likelihood = self.likelihood(pose_measurements[i], measurement, self.model_params.hit_sigma)
                    likelihood = likelihood * measurement_likelihood

        return likelihood

    def likelihood(self, x, mean, sigma):
        return 1. / (sigma * sqrt(2*pi)) * exp(-0.5 * (x - mean) * (x - mean)/(sigma * sigma))
