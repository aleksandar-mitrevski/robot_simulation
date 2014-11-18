class SensorMeasurements(object):
    '''Defines a structure for storing seven measurements of two distance sensors
    corresponding to the front, left, and right directions, two back diagonals, and two right diagonals.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, front=0., left_diagonal=0., left_back_diagonal=0., left=0., right_diagonal=0., right_back_diagonal=0., right=0.):
        self.front = front
        self.left_diagonal = left_diagonal
        self.left_back_diagonal = left_back_diagonal
        self.left = left
        self.right_diagonal = right_diagonal
        self.right_back_diagonal = right_back_diagonal
        self.right = right

    def less_than(self, threshold):
        '''Returns True if the front measurement is less than 'threshold' and False otherwise.
        '''
        return self.front < threshold
