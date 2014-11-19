from random import uniform
from copy import copy, deepcopy

from particle import Particle
from motion_model import MotionModel
from measurement_model import MeasurementModel

class ParticleFilter(object):
    '''Defines a utility for localising a robot using a particle filter.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, motion_model_noise_params, measurement_model_params, filter_params, measurement_generator_callback):
        self.filter_params = copy(filter_params)
        self.motion_model = MotionModel(motion_model_noise_params)
        self.measurement_model = MeasurementModel(measurement_model_params, measurement_generator_callback)
        self.particles = list()
        self.generate_new_particle_set()

    def iterate_filter(self, motion_command, measurements):
        '''Performs an update of the filter with the given motion command
        and set of sensor measurements.

        Keyword arguments:
        motion_command -- A 'velocity.Velocity' object.
        measurements -- A dictionary where each key is a name of a sensor frame and the values are measurements obtained with the respective sensor.

        '''
        self.move_particles(motion_command)
        resample = self.update_particle_weights(measurements)
        if resample:
            self.particles = self.resample_particles()

    def move_particles(self, motion_command):
        '''Moves all particles based on the given motion command.

        Keyword arguments:
        motion_command -- A 'velocity.Velocity' object.

        '''
        for i in xrange(self.filter_params.number_of_particles):
            self.particles[i].pose = self.motion_model.sample_motion_model(self.particles[i].pose, motion_command)

    def update_particle_weights(self, measurements):
        '''Updates the particle weights based on the given sensor measurements.

        Keyword arguments:
        measurements -- A dictionary where each key is a name of a sensor frame and the values are measurements obtained with the respective sensor.

        '''
        weight_sum = 0.
        for i in xrange(self.filter_params.number_of_particles):
            self.particles[i].weight = self.measurement_model.calculate_measurement_likelihood(self.particles[i].pose, measurements)
            weight_sum = weight_sum + self.particles[i].weight

        resample = True
        if weight_sum < self.filter_params.weight_sum_tolerance:
            resample = False
            self.generate_new_particle_set()
        else:
            for i in xrange(self.filter_params.number_of_particles):
                self.particles[i].weight = self.particles[i].weight / weight_sum
        return resample

    def resample_particles(self):
        '''Performs an importance particle resampling step based on the current particle weights.
        '''
        number_resampled_particles = self.filter_params.number_of_particles - self.filter_params.number_of_random_particles
        step = 1. / number_resampled_particles
        start_weight = uniform(0, step)
        current_particle = 0
        current_weight = self.particles[0].weight
        weight_sum = 0.

        #we add random particles to the particle set
        new_particles = list()
        for i in xrange(self.filter_params.number_of_random_particles):
            particle = Particle.random_particle(self.filter_params.neg_x_limit, self.filter_params.x_limit, self.filter_params.neg_y_limit, self.filter_params.y_limit, 1/self.filter_params.number_of_particles)
            weight_sum = weight_sum + particle.weight
            new_particles.append(particle)

        #we resample particles using low variance resampling
        for i in xrange(number_resampled_particles):
            weight = start_weight + i * step
            while weight > current_weight:
                current_particle = current_particle + 1
                current_weight = current_weight + self.particles[current_particle].weight
            particle = deepcopy(self.particles[current_particle])
            weight_sum = weight_sum + particle.weight
            new_particles.append(particle)

        for i in xrange(self.filter_params.number_of_particles):
            new_particles[i].weight = new_particles[i].weight / weight_sum

        return new_particles

    def generate_new_particle_set(self):
        '''Discards the current particle set and creates a new set of random particles.
        '''
        self.particles = list()
        for i in xrange(self.filter_params.number_of_particles):
            self.particles.append(Particle.random_particle(self.filter_params.neg_x_limit, self.filter_params.x_limit, self.filter_params.neg_y_limit, self.filter_params.y_limit, 1./self.filter_params.number_of_particles))

    def get_most_likely_pose(self):
        '''Returns the pose of the particle with the largest weight.
        '''
        max_weight_particle = 0
        max_weight = self.particles[0].weight

        for i,particle in enumerate(self.particles):
            if particle.weight > max_weight:
                max_weight_particle = i
                max_weight = particle.weight

        return copy(self.particles[max_weight_particle].pose)
