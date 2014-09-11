from particle import Particle
from motion_model import MotionModel

class ParticleFilter(object):
    def __init__(self, alpha1, alpha2, alpha3, alpha4, number_of_particles=100):
        self.number_of_articles = number_of_particles
        self.particles = [None] * number_of_particles
        self.motion_model = MotionModel(alpha1, alpha2, alpha3, alpha4)
        for i in xrange(number_of_particles):
            self.particles[i] = Particles(0, 0, 0, 1./number_of_particles)

    def move_particles(self, motion_command):
        for i in xrange(self.number_of_particles):
            self.particles[i].pose = self.motion_model(self.particles.pose[i], motion_command)

    def update_particle_weights(self, measurements):
        pass
