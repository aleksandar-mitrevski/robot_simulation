class MotionModelNoiseParameters(object):
    def __init__(self, alpha1, alpha2, alpha3, alpha4):
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.alpha3 = alpha3
        self.alpha4 = alpha4

    def __copy__(self):
        return MotionModelNoiseParameters(self.alpha1, self.alpha2, self.alpha3, self.alpha4)

class MeasurementModelParameters(object):
    def __init__(self, min_range, max_range, angle_increment, hit_sigma):
        self.min_range = min_range
        self.max_range = max_range
        self.angle_increment = angle_increment
        self.hit_sigma = hit_sigma

    def __copy__(self):
        return MeasurementModelParameters(self.min_range, self.max_range, self.angle_increment, self.hit_sigma)

class FilterParameters(object):
    def __init__(self, neg_x_limit, x_limit, neg_y_limit, y_limit, number_of_particles, number_of_random_particles):
        self.neg_x_limit = neg_x_limit
        self.x_limit = x_limit
        self.neg_y_limit = neg_y_limit
        self.y_limit = y_limit
        self.number_of_particles = number_of_particles
        self.number_of_random_particles = number_of_random_particles

    def __copy__(self):
        return FilterParameters(self.neg_x_limit, self.x_limit, self.neg_y_limit, self.y_limit, self.number_of_particles, self.number_of_random_particles)
